import os
from types import SimpleNamespace

import numpy as np

from robot_backend_hardware.common.camera.capture import CameraConfig, FFmpegV4L2Stream


def test_ffmpeg_pipe_decodes_complete_rgb_frames():
    stream = FFmpegV4L2Stream(CameraConfig(name="test", backend="ffmpeg_v4l2", width=2, height=1))
    stream._running = True
    read_fd, write_fd = os.pipe()
    os.write(write_fd, bytes(range(12)))
    os.close(write_fd)
    with os.fdopen(read_fd, "rb", buffering=0) as pipe:
        stream._read_frames(SimpleNamespace(stdout=pipe, poll=lambda: None))
    assert stream.frame_count == 2
    np.testing.assert_array_equal(stream.get_frame(), np.array([[[6, 7, 8], [9, 10, 11]]], dtype=np.uint8))
    copied = stream.get_frame()
    copied[:] = 0
    assert stream.get_frame()[0, 0, 0] == 6


def test_ffmpeg_process_and_pipes_are_reaped():
    events = []

    class Process:
        stdout = SimpleNamespace(close=lambda: events.append("stdout.close"))
        stderr = SimpleNamespace(close=lambda: events.append("stderr.close"))

        def poll(self):
            return None

        def terminate(self):
            events.append("terminate")

        def wait(self, timeout):
            events.append("wait")

    stream = FFmpegV4L2Stream(CameraConfig(name="test", backend="ffmpeg_v4l2"))
    process = Process()
    stream._process = process
    stream._reap_process(process)
    assert events == ["terminate", "wait", "stdout.close", "stderr.close"]
    assert stream._process is None
