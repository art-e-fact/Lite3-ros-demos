"""Newton follow camera keeps the mp4 frame size fixed when the viewer window resizes.

On macOS the viewer framebuffer can change size mid-run (Retina backing-scale change when
the window moves between displays, or a user resize).  imageio then raises
``ValueError: All images in a movie should have same size`` and the simulation dies.
The recorder must lock its output size at start and resample later frames instead.
"""

import sys
from pathlib import Path
from types import SimpleNamespace

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'simulation_package'))

from sensors.newton.follow_camera_recorder import NewtonFollowCameraRecorder  # noqa: E402,I100
from simulation_config import FollowCameraConfig  # noqa: E402,I100


class _FakeViewer:
    """Window opened at 64x48 whose framebuffer jumps to 2x after the first frame."""

    renderer = SimpleNamespace(window=SimpleNamespace(get_framebuffer_size=lambda: (64, 48)))

    def __init__(self):
        self.calls = 0

    def get_frame(self):
        self.calls += 1
        shape = (48, 64, 3) if self.calls == 1 else (96, 128, 3)
        return SimpleNamespace(numpy=lambda: np.zeros(shape, dtype=np.uint8))

    def set_camera(self, *args):
        pass

    def begin_frame(self, timestamp):
        pass

    def log_state(self, state):
        pass

    def end_frame(self):
        pass


def test_resamples_frames_to_locked_size(tmp_path):
    """Frames of a different size are resampled to the size the writer was opened with."""
    config = FollowCameraConfig(enabled=True, video_path=str(tmp_path / 'out.mp4'))
    recorder = NewtonFollowCameraRecorder(object(), None, config, viewer=_FakeViewer())
    assert recorder.frame_size == (48, 64)

    shapes = []
    recorder.writer = SimpleNamespace(  # no ffmpeg needed
        append_data=lambda frame: shapes.append(frame.shape[:2]), close=lambda: None
    )
    body_q = np.array([[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]])  # identity pose, quat xyzw
    state = SimpleNamespace(body_q=SimpleNamespace(numpy=lambda: body_q))

    for _ in range(3):
        recorder.update(state)
    recorder.close()

    assert recorder.frame_count == 3
    assert shapes == [(48, 64)] * 3
