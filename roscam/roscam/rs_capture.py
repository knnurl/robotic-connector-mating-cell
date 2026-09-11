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
Frame = namedtuple('Frame', 'bgr depth_m t_host')

Intrinsics = namedtuple('Intrinsics', 'fx fy cx cy coeffs width height')


class RsCapture:
    """Owns the RealSense device. ONE instance per camera per machine -
    a second pipeline on the same device will fail to start."""

    def __init__(self, width=640, height=480, fps=15, enable_depth=False,
                 serial='', auto_recover=True, timeouts_before_reset=8,
                 max_resets=5):
        self.width = int(width)
        self.height = int(height)
        self.fps = int(fps)
        self.enable_depth = bool(enable_depth)
        self.serial = str(serial)
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

        intr = profile.get_stream(rs.stream.color) \
            .as_video_stream_profile().get_intrinsics()
        self.intrinsics = Intrinsics(intr.fx, intr.fy, intr.ppx, intr.ppy,
                                     list(intr.coeffs), intr.width, intr.height)
        return self.intrinsics

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
        t_host = time.time()
        if self._align is not None:
            frames = self._align.process(frames)
        color = frames.get_color_frame()
        if not color:
            return None
        bgr = np.asanyarray(color.get_data()).copy()
        depth_m = None
        if self.enable_depth:
            depth = frames.get_depth_frame()
            if depth:
                depth_m = (np.asanyarray(depth.get_data())
                           .astype(np.float32) * self._depth_scale)
        return Frame(bgr, depth_m, t_host)

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
