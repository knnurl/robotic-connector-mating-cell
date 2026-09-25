#!/usr/bin/env python3
"""Record the camera's own frames, inside the process that owns the camera.

Images never go on DDS on this cell (they contend with the 1 kHz FCI loop),
so recording happens where the frames already are: vision_standalone's loop
hands each frame here, and a writer thread puts it on disk. PERCEPTION_PLAN
Phase 0 uses the recordings as ground truth (the ArUco poses published for
the same frame are stored with it) and to compare capture settings.

A session directory holds:

    session.json     capture settings, intrinsics, marker ids and sizes;
                     frames_written / frames_dropped added at stop
    frames.jsonl     one line per frame: i, t_host (s), t_hw (ms, SDK),
                     t_domain, stamp (s, the image header stamp), and the
                     poses published for that frame as {topic: {stamp, t, q}}
    color/NNNNNN.png lossless BGR
    depth/NNNNNN.png uint16 depth exactly as the camera sent it; metres =
                     value * depth_scale (session.json)

No ROS imports: load_session() is shared by the offline tools
(tools/fr3/vision) and the tests.
"""

import datetime
import json
import pathlib
import queue
import threading
from collections import namedtuple

import cv2
import numpy as np

Recorded = namedtuple('Recorded', 'i bgr depth_m depth_raw line')

_STOP = object()


class FrameRecorder:
    """start() / record() / stop(). record() never blocks the camera loop:
    a full queue (max_queue frames) drops the frame and counts it."""

    def __init__(self, max_queue=30):
        self._max_queue = int(max_queue)
        self._q = queue.Queue(maxsize=self._max_queue)
        self._thread = None
        self._lock = threading.Lock()
        self._count_lock = threading.Lock()     # written/dropped: two threads count
        self.dir = None
        self.written = 0
        self.dropped = 0
        self.error = None
        self._next = 0

    @property
    def recording(self):
        return self.dir is not None

    def start(self, session_dir, meta):
        """Begin a session in session_dir (created; must not already hold
        one). meta goes to session.json as-is, plus 'started'."""
        with self._lock:
            if self.dir is not None:
                self._stop_locked()
            path = pathlib.Path(session_dir)
            (path / 'color').mkdir(parents=True, exist_ok=True)
            (path / 'depth').mkdir(exist_ok=True)
            if (path / 'frames.jsonl').exists():
                raise FileExistsError(f'{path} already holds a recording')
            meta = dict(meta)
            meta['started'] = datetime.datetime.now().isoformat(timespec='milliseconds')
            (path / 'session.json').write_text(json.dumps(meta, indent=1, default=_plain))
            self._lines = (path / 'frames.jsonl').open('x')
            # A fresh queue per session: a frame that raced the last stop()
            # stays in the old queue instead of landing in this recording.
            self._q = queue.Queue(maxsize=self._max_queue)
            self.dir, self.written, self.dropped, self.error, self._next = path, 0, 0, None, 0
            self._thread = threading.Thread(target=self._writer, args=(self._q,),
                                            name='frame_recorder', daemon=True)
            self._thread.start()
        return path

    def record(self, frame, stamp_s, poses):
        """Queue one rs_capture.Frame with its image stamp and the poses
        published for it ({topic: (stamp_s, t, q)})."""
        q = self._q
        if self.dir is None:
            return
        item = (self._next, frame, float(stamp_s), dict(poses))
        self._next += 1
        try:
            q.put_nowait(item)
        except queue.Full:
            with self._count_lock:
                self.dropped += 1

    def stop(self):
        """Flush, close and write the counts into session.json."""
        with self._lock:
            return self._stop_locked()

    def _stop_locked(self):
        if self.dir is None:
            return None
        self._q.put(_STOP)
        self._thread.join()
        self._lines.close()
        path = self.dir / 'session.json'
        meta = json.loads(path.read_text())
        meta.update(ended=datetime.datetime.now().isoformat(timespec='milliseconds'),
                    frames_written=self.written, frames_dropped=self.dropped,
                    writer_error=self.error)
        path.write_text(json.dumps(meta, indent=1, default=_plain))
        summary = {'dir': str(self.dir), 'written': self.written, 'dropped': self.dropped,
                   'error': self.error}
        self.dir = None
        return summary

    def _writer(self, q):
        while True:
            item = q.get()
            if item is _STOP:
                return
            i, frame, stamp, poses = item
            try:
                name = f'{i:06d}.png'
                if not cv2.imwrite(str(self.dir / 'color' / name), frame.bgr):
                    raise OSError('colour PNG not written')
                has_depth = frame.depth_raw is not None
                if has_depth and not cv2.imwrite(str(self.dir / 'depth' / name),
                                                 np.ascontiguousarray(frame.depth_raw,
                                                                      dtype=np.uint16)):
                    raise OSError('depth PNG not written')
                line = {'i': i, 't_host': frame.t_host, 't_hw': frame.t_hw,
                        't_domain': frame.t_domain, 'stamp': stamp, 'depth': has_depth,
                        'poses': {topic: {'stamp': p[0], 't': list(map(float, p[1])),
                                          'q': list(map(float, p[2]))}
                                  for topic, p in poses.items()}}
                self._lines.write(json.dumps(line) + '\n')
                self._lines.flush()
                with self._count_lock:
                    self.written += 1
            except Exception as e:                          # noqa: BLE001
                self.error = f'frame {i}: {e}'
                with self._count_lock:
                    self.dropped += 1


def load_session(session_dir):
    """(meta, generator of Recorded) for a session directory. Frames are
    read lazily, in recording order."""
    path = pathlib.Path(session_dir)
    meta = json.loads((path / 'session.json').read_text())
    scale = float(meta.get('capture', {}).get('depth_scale', meta.get('depth_scale', 1.0)))

    def frames():
        with (path / 'frames.jsonl').open() as f:
            for text in f:
                line = json.loads(text)
                name = f"{line['i']:06d}.png"
                bgr = cv2.imread(str(path / 'color' / name), cv2.IMREAD_UNCHANGED)
                depth_raw = depth_m = None
                if line.get('depth'):
                    depth_raw = cv2.imread(str(path / 'depth' / name), cv2.IMREAD_UNCHANGED)
                    depth_m = depth_raw.astype(np.float32) * scale
                yield Recorded(line['i'], bgr, depth_m, depth_raw, line)
    return meta, frames()


def _plain(v):
    """json default: numpy scalars and arrays."""
    return np.asarray(v).tolist()
