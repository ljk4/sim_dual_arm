"""Tests for visual-tactile V4L2 controls."""

from __future__ import annotations

from robot_backend_hardware.common.camera.capture import (
    CameraConfig,
    FFmpegV4L2Stream,
    MultiCameraCapture,
    V4L2Stream,
)


class _FakeCV2:
    CAP_PROP_AUTO_EXPOSURE = 21
    CAP_PROP_EXPOSURE = 15


class _FakeCapture:
    def __init__(self) -> None:
        self.values = {}

    def set(self, prop: int, value: float) -> bool:
        self.values[prop] = value
        return True

    def get(self, prop: int) -> float:
        return self.values[prop]


def test_manual_exposure_controls_are_applied() -> None:
    stream = V4L2Stream(CameraConfig(
        name="tactile_test",
        backend="v4l2",
        auto_exposure=False,
        exposure_time_absolute=166,
    ))
    capture = _FakeCapture()

    assert stream._apply_capture_controls(capture, _FakeCV2)
    assert capture.values == {
        _FakeCV2.CAP_PROP_AUTO_EXPOSURE: 1.0,
        _FakeCV2.CAP_PROP_EXPOSURE: 166.0,
    }


def test_auto_exposure_control_is_applied_without_manual_value() -> None:
    stream = V4L2Stream(CameraConfig(
        name="tactile_test",
        backend="v4l2",
        auto_exposure=True,
    ))
    capture = _FakeCapture()

    assert stream._apply_capture_controls(capture, _FakeCV2)
    assert capture.values == {
        _FakeCV2.CAP_PROP_AUTO_EXPOSURE: 3.0,
    }


def test_multicamera_selects_ffmpeg_backend() -> None:
    capture = MultiCameraCapture([
        CameraConfig(name="tactile_test", backend="ffmpeg_v4l2"),
    ])

    assert isinstance(capture._streams[0], FFmpegV4L2Stream)


def test_ffmpeg_outputs_rgb_frames_at_configured_shape() -> None:
    stream = FFmpegV4L2Stream(CameraConfig(
        name="tactile_test",
        device="/dev/video-test",
        width=640,
        height=480,
        fps=30,
        backend="ffmpeg_v4l2",
    ))
    stream._ffmpeg = "/usr/bin/ffmpeg"

    command = stream._ffmpeg_command("/dev/video20")
    assert command[0] == "/usr/bin/ffmpeg"
    assert command[command.index("-input_format") + 1] == "mjpeg"
    assert command[command.index("-video_size") + 1] == "640x480"
    assert command[command.index("-framerate") + 1] == "30"
    assert command[command.index("-pix_fmt") + 1] == "rgb24"
