#!/usr/bin/env python3
"""RealSense capture OUTSIDE the ROS 2 graph. Pure pyrealsense2 + numpy.

Why this exists: raw image/depth topics are megabytes per second of DDS
traffic, and on an FR3 cell that traffic contends with the 1 kHz FCI loop
(communication_constraints_violation). The fix that beats all QoS tuning is
to never put frames on the graph at all: capture in-process, run the vision
math, and let only poses (~2 KB/s) cross into ROS. This module is the
capture half of that pattern; cam_pub / connector_pose / vision_standalone
consume it via `source:=realsense` (or `external`).

No ROS imports here - pyrealsense2 is imported lazily in start(), so the
roscam package works on machines without the RealSense SDK.

Standalone self-test (no ROS, no robot - just proves the camera path):

    python3 -m roscam.rs_capture --seconds 5 [--depth]

prints achieved FPS, inter-frame jitter, and the DDS bandwidth this
configuration keeps OFF the network.
"""

import time
from collections import namedtuple

import numpy as np

# bgr: HxWx3 uint8. depth_m: HxW float32 metres aligned to colour (or None).
# t_host: host wall-clock (time.time()) at frame arrival.
# t_hw / t_domain: the colour frame's SDK timestamp (ms) and its domain
# (hardware clock, or the host's when the firmware cannot stamp).
# depth_raw / depth_scale: the aligned depth as the uint16 the camera sent,
# and metres per unit - what a lossless recording keeps.
Frame = namedtuple('Frame',
                   'bgr depth_m t_host t_hw t_domain depth_raw depth_scale exposure_us gain',
                   defaults=(None, '', None, 1.0, None, None))

Intrinsics = namedtuple('Intrinsics', 'fx fy cx cy coeffs width height')


class RsCapture:
    """Owns the RealSense device. ONE instance per camera per machine -
    a second pipeline on the same device will fail to start."""

    def __init__(self, width=640, height=480, fps=15, enable_depth=False,
                 serial='', auto_recover=True, timeouts_before_reset=8,
                 max_resets=5, frames_to_forgive=200, preset='',
                 spatial_filter=False, exposure_us=-1, gain=-1):
        self.width = int(width)
        self.height = int(height)
        self.fps = int(fps)
        self.enable_depth = bool(enable_depth)
        self.serial = str(serial)
        # Depth-quality settings (PERCEPTION_PLAN Phase 0): the D405 visual
        # preset by name ('' = leave the device's), the SDK spatial filter on
        # the aligned depth (no temporal filter: it adds lag), and a locked
        # exposure in microseconds (-1 = auto), and with it a sensor gain (-1 =
        # leave the device's). On the D405 colour and depth come from the
        # same sensor: both change together. set_exposure() / set_gain()
        # change them while streaming.
        self.preset = str(preset)
        self.spatial_filter = bool(spatial_filter)
        self.exposure_us = int(exposure_us)
        self.gain = int(gain)
        self._spatial = None
        self._rs = None
        self._pipeline = None
        self._align = None
        self._depth_scale = 1.0
        self.intrinsics = None
        # Self-healing. The D405 re-enumerates when the kernel autosuspends
        # it (power/control=auto, 2 s delay by default): the device comes
        # back with a NEW USB device number and the handle this object holds
        # is dead, so wait_for_frames times out forever with no error. Seen
        # three times on this cell (device 003 -> 005 -> 008). A
        # hardware_reset + fresh pipeline recovers it.
        self.auto_recover = bool(auto_recover)
        self.timeouts_before_reset = int(timeouts_before_reset)
        self.max_resets = int(max_resets)
        self.consecutive_timeouts = 0
        self.resets_done = 0
        # max_resets must mean "recoveries that did not stick", not
        # "recoveries ever". A flexing USB cable on a moving wrist drops the
        # device repeatedly over a long session; each recovery genuinely
        # works, so a lifetime counter retires a perfectly healthy camera
        # mid-run. Streaming this many frames after a reset clears the
        # budget, so only a camera that will NOT stay up exhausts it.
        self.frames_to_forgive = int(frames_to_forgive)
        self._frames_since_reset = 0
        self.on_event = None          # optional callback(str) for logging

    def start(self):
        """Start streaming. Returns Intrinsics of the colour stream."""
        try:
            import pyrealsense2 as rs
        except ImportError as e:
            raise RuntimeError(
                'pyrealsense2 is required for source:=realsense '
                '(pip install pyrealsense2). Topic mode needs no SDK.') from e
        self._rs = rs
        cfg = rs.config()
        if self.serial:
            cfg.enable_device(self.serial)
        cfg.enable_stream(rs.stream.color, self.width, self.height,
                          rs.format.bgr8, self.fps)
        if self.enable_depth:
            cfg.enable_stream(rs.stream.depth, self.width, self.height,
                              rs.format.z16, self.fps)
            self._align = rs.align(rs.stream.color)

        self._pipeline = rs.pipeline()
        profile = self._pipeline.start(cfg)

        if self.enable_depth:
            sensor = profile.get_device().first_depth_sensor()
            self._depth_scale = float(sensor.get_depth_scale())
        self._apply_settings(profile.get_device())

        intr = profile.get_stream(rs.stream.color) \
            .as_video_stream_profile().get_intrinsics()
        self.intrinsics = Intrinsics(intr.fx, intr.fy, intr.ppx, intr.ppy,
                                     list(intr.coeffs), intr.width, intr.height)
        return self.intrinsics

    def _apply_settings(self, device):
        """Preset, exposure and spatial filter; re-applied after every
        recovery, since start() runs again."""
        rs = self._rs
        if self.preset and self.enable_depth:
            ds = device.first_depth_sensor()
            if not ds.supports(rs.option.visual_preset):
                raise RuntimeError('this camera has no visual presets')
            rng = ds.get_option_range(rs.option.visual_preset)
            names = {ds.get_option_value_description(rs.option.visual_preset, i).lower(): i
                     for i in range(int(rng.min), int(rng.max) + 1)}
            if self.preset.lower() not in names:
                raise RuntimeError(f'unknown visual preset {self.preset!r}; this camera has '
                                   f'{sorted(names)}')
            ds.set_option(rs.option.visual_preset, names[self.preset.lower()])
        if self.exposure_us > 0:
            self._set_exposure(device, self.exposure_us)
        if self.gain >= 0:
            self._set_gain(device, self.gain)
        self._spatial = rs.spatial_filter() if (self.spatial_filter and self.enable_depth) \
            else None

    def _sensors(self, device, option):
        return [s for s in device.query_sensors() if s.supports(option)]

    def _set_exposure(self, device, exposure_us):
        rs = self._rs
        sensors = self._sensors(device, rs.option.exposure)
        if not sensors:
            raise RuntimeError('this camera has no settable exposure')
        for sensor in sensors:
            if exposure_us > 0:
                if sensor.supports(rs.option.enable_auto_exposure):
                    sensor.set_option(rs.option.enable_auto_exposure, 0)
                sensor.set_option(rs.option.exposure, float(exposure_us))
            elif sensor.supports(rs.option.enable_auto_exposure):
                sensor.set_option(rs.option.enable_auto_exposure, 1)

    def _set_gain(self, device, gain):
        sensors = self._sensors(device, self._rs.option.gain)
        if not sensors:
            raise RuntimeError('this camera has no settable gain')
        for sensor in sensors:
            sensor.set_option(self._rs.option.gain, float(gain))

    def set_exposure(self, exposure_us):
        """While streaming: a locked exposure in us, or <= 0 for auto."""
        self.exposure_us = int(exposure_us) if exposure_us > 0 else -1
        if self._pipeline is not None:
            self._set_exposure(self._pipeline.get_active_profile().get_device(), exposure_us)

    def set_gain(self, gain):
        """While streaming: the sensor gain (a locked exposure keeps it)."""
        self.gain = int(gain)
        if self._pipeline is not None and gain >= 0:
            self._set_gain(self._pipeline.get_active_profile().get_device(), gain)

    def settings(self):
        """What this capture runs with - for a recording's session.json."""
        return {'width': self.width, 'height': self.height, 'fps': self.fps,
                'depth': self.enable_depth, 'preset': self.preset or 'device default',
                'spatial_filter': self.spatial_filter,
                'exposure_us': self.exposure_us if self.exposure_us > 0 else 'auto',
                'gain': self.gain if self.gain >= 0 else 'device',
                'depth_scale': self._depth_scale}

    def _emit(self, msg):
        if self.on_event is not None:
            try:
                self.on_event(msg)
            except Exception:                               # noqa: BLE001
                pass

    def recover(self):
        """Power-cycle the device and rebuild the pipeline.

        Returns True if streaming again. Safe to call repeatedly; bounded by
        max_resets so a genuinely unplugged camera does not spin forever.
        """
        if self._rs is None:
            return False
        rs = self._rs
        self.resets_done += 1
        self._emit(f'camera recovery attempt {self.resets_done}'
                   f'/{self.max_resets}: hardware_reset')
        try:
            if self._pipeline is not None:
                try:
                    self._pipeline.stop()
                except Exception:                           # noqa: BLE001
                    pass
                self._pipeline = None
            devs = rs.context().query_devices()
            if len(devs) > 0:
                devs[0].hardware_reset()
            # wait for re-enumeration
            for _ in range(30):
                time.sleep(1.0)
                try:
                    if len(rs.context().query_devices()) > 0:
                        break
                except Exception:                           # noqa: BLE001
                    pass
            time.sleep(2.0)
            self.start()
        except Exception as e:                              # noqa: BLE001
            self._emit(f'camera recovery FAILED: {e}')
            return False
        self.consecutive_timeouts = 0
        self._frames_since_reset = 0
        self._emit('camera recovered')
        return True

    def wait_frame(self, timeout_s=1.0):
        """Block for the next frame set. Returns a Frame, or None on timeout.

        After timeouts_before_reset consecutive timeouts this power-cycles
        the camera (see auto_recover): a suspended/re-enumerated D405 leaves
        a dead handle that never produces another frame on its own.
        """
        if self._pipeline is None:
            return None
        try:
            frames = self._pipeline.wait_for_frames(int(timeout_s * 1000))
        except RuntimeError:
            self.consecutive_timeouts += 1
            if (self.auto_recover
                    and self.consecutive_timeouts >= self.timeouts_before_reset
                    and self.resets_done < self.max_resets):
                self.recover()
            elif (self.consecutive_timeouts == self.timeouts_before_reset
                  and self.resets_done >= self.max_resets):
                self._emit(f'camera dead after {self.max_resets} resets - '
                           'check USB autosuspend (power/control) and cabling')
            return None  # timeout
        self.consecutive_timeouts = 0
        if self.resets_done:
            self._frames_since_reset += 1
            if self._frames_since_reset >= self.frames_to_forgive:
                self._emit(f'camera stable for {self._frames_since_reset} '
                           f'frames - clearing reset budget '
                           f'(was {self.resets_done}/{self.max_resets})')
                self.resets_done = 0
                self._frames_since_reset = 0
        t_host = time.time()
        if self._align is not None:
            frames = self._align.process(frames)
        color = frames.get_color_frame()
        if not color:
            return None
        bgr = np.asanyarray(color.get_data()).copy()
        t_hw = float(color.get_timestamp())
        t_domain = str(color.get_frame_timestamp_domain())
        # What this frame was actually shot with (auto exposure changes it).
        md = self._rs.frame_metadata_value
        exposure_us = (float(color.get_frame_metadata(md.actual_exposure))
                       if color.supports_frame_metadata(md.actual_exposure) else None)
        gain = (float(color.get_frame_metadata(md.gain_level))
                if color.supports_frame_metadata(md.gain_level) else None)
        depth_m = depth_raw = None
        if self.enable_depth:
            depth = frames.get_depth_frame()
            if depth and self._spatial is not None:
                depth = self._spatial.process(depth).as_depth_frame()
            if depth:
                depth_raw = np.asanyarray(depth.get_data()).copy()
                depth_m = depth_raw.astype(np.float32) * self._depth_scale
        return Frame(bgr, depth_m, t_host, t_hw, t_domain, depth_raw, self._depth_scale,
                     exposure_us, gain)

    def stop(self):
        if self._pipeline is not None:
            self._pipeline.stop()
            self._pipeline = None


def main():
    import argparse
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seconds', type=float, default=5.0)
    parser.add_argument('--width', type=int, default=640)
    parser.add_argument('--height', type=int, default=480)
    parser.add_argument('--fps', type=int, default=15)
    parser.add_argument('--depth', action='store_true')
    parser.add_argument('--serial', default='')
    args = parser.parse_args()

    cap = RsCapture(args.width, args.height, args.fps,
                    enable_depth=args.depth, serial=args.serial)
    intr = cap.start()
    print(f'colour intrinsics: fx={intr.fx:.1f} fy={intr.fy:.1f} '
          f'cx={intr.cx:.1f} cy={intr.cy:.1f} ({intr.width}x{intr.height})')

    arrivals = []
    t_end = time.time() + args.seconds
    try:
        while time.time() < t_end:
            frame = cap.wait_frame()
            if frame is not None:
                arrivals.append(frame.t_host)
    finally:
        cap.stop()

    if len(arrivals) < 2:
        print('FAILED: fewer than 2 frames captured')
        raise SystemExit(1)
    gaps = np.diff(arrivals) * 1000.0
    fps = (len(arrivals) - 1) / (arrivals[-1] - arrivals[0])
    frame_bytes = args.width * args.height * 3
    if args.depth:
        frame_bytes += args.width * args.height * 2
    print(f'{len(arrivals)} frames, {fps:.1f} fps '
          f'(target {args.fps}); inter-frame gap '
          f'avg {gaps.mean():.1f} / max {gaps.max():.1f} ms')
    print(f'kept OFF the DDS graph: ~{fps * frame_bytes / 1e6:.1f} MB/s')
    print('SELF-TEST OK')


if __name__ == '__main__':
    main()
