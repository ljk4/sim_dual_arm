"""Parallel RealSense and visual-tactile UVC camera capture."""

import threading
import time
import logging
import os
import re
import select
import shutil
import subprocess
from typing import Literal, Optional

import numpy as np
from pydantic import Field

from robot_backend_hardware.common.config import DeviceSettings

log = logging.getLogger(__name__)


class CameraConfig(DeviceSettings):
    name: str = Field(min_length=1)
    serial_number: str = ""
    width: int = Field(default=640, gt=0)
    height: int = Field(default=480, gt=0)
    fps: int = Field(default=30, gt=0)
    enable_depth: bool = False
    device: str = ""
    backend: Literal["realsense", "v4l2", "ffmpeg_v4l2"] = "realsense"
    intrinsics: Optional[dict] = None
    auto_exposure: Optional[bool] = None
    exposure_time_absolute: Optional[int] = None


class RealSenseStream:
    """Single RealSense camera capture thread identified by serial number."""

    def __init__(self, config: CameraConfig):
        self._config = config
        self._pipeline = None
        self._frame: Optional[np.ndarray] = None
        self._depth_frame: Optional[np.ndarray] = None
        self._lock = threading.Lock()
        self._running = False
        self._thread: Optional[threading.Thread] = None
        self._frame_count = 0
        self._intrinsics: dict = {}

    @property
    def name(self) -> str:
        return self._config.name

    @property
    def frame_count(self) -> int:
        return self._frame_count

    def start(self) -> bool:
        try:
            import pyrealsense2 as rs
        except ImportError:
            log.error("pyrealsense2 not installed")
            return False

        try:
            self._pipeline = rs.pipeline()
            cfg = rs.config()
            cfg.enable_device(self._config.serial_number)
            cfg.enable_stream(
                rs.stream.color,
                self._config.width,
                self._config.height,
                rs.format.rgb8,
                self._config.fps,
            )
            if self._config.enable_depth:
                cfg.enable_stream(
                    rs.stream.depth,
                    self._config.width,
                    self._config.height,
                    rs.format.z16,
                    self._config.fps,
                )

            profile = self._pipeline.start(cfg)
            intr = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
            self._intrinsics = {
                "backend": "realsense",
                "serial_number": self._config.serial_number,
                "width": int(intr.width),
                "height": int(intr.height),
                "fps": self._config.fps,
                "calibration_status": "factory",
                "camera_matrix": [
                    [float(intr.fx), 0.0, float(intr.ppx)],
                    [0.0, float(intr.fy), float(intr.ppy)],
                    [0.0, 0.0, 1.0],
                ],
                "distortion_model": str(intr.model),
                "distortion_coefficients": [float(v) for v in intr.coeffs],
            }
            log.info(
                "RealSense '%s' (SN:%s) started at %dx%d@%dfps",
                self._config.name,
                self._config.serial_number,
                self._config.width,
                self._config.height,
                self._config.fps,
            )
        except Exception as e:
            log.error(
                "Failed to start RealSense '%s' (SN:%s): %s",
                self._config.name,
                self._config.serial_number,
                e,
            )
            self._pipeline = None
            return False

        self._running = True
        self._thread = threading.Thread(
            target=self._capture_loop, daemon=True, name=f"cam-{self._config.name}"
        )
        self._thread.start()
        return True

    def stop(self):
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=3.0)
        if self._pipeline is not None:
            try:
                self._pipeline.stop()
            except Exception:
                pass
            self._pipeline = None

    def get_frame(self) -> Optional[np.ndarray]:
        with self._lock:
            return self._frame.copy() if self._frame is not None else None

    def get_depth(self) -> Optional[np.ndarray]:
        with self._lock:
            return self._depth_frame.copy() if self._depth_frame is not None else None

    def get_intrinsics(self) -> dict:
        return dict(self._intrinsics)

    def _capture_loop(self):
        while self._running:
            try:
                frames = self._pipeline.wait_for_frames(timeout_ms=1000)
                color = frames.get_color_frame()
                if color:
                    img = np.asanyarray(color.get_data())
                    with self._lock:
                        self._frame = img
                    self._frame_count += 1

                if self._config.enable_depth:
                    depth = frames.get_depth_frame()
                    if depth:
                        with self._lock:
                            self._depth_frame = np.asanyarray(depth.get_data())
            except Exception as e:
                if self._running:
                    log.warning("Frame capture error on '%s': %s", self._config.name, e)
                    time.sleep(0.01)


class V4L2Stream:
    """Single UVC camera opened through a stable /dev/v4l/by-path link."""

    def __init__(self, config: CameraConfig):
        self._config = config
        self._capture = None
        self._frame: Optional[np.ndarray] = None
        self._lock = threading.Lock()
        self._running = False
        self._thread: Optional[threading.Thread] = None
        self._frame_count = 0

    @property
    def name(self) -> str:
        return self._config.name

    @property
    def frame_count(self) -> int:
        return self._frame_count

    def start(self) -> bool:
        try:
            import cv2
        except ImportError:
            log.error("opencv-python not installed")
            return False
        device = os.path.realpath(self._config.device)
        cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self._config.width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self._config.height)
        cap.set(cv2.CAP_PROP_FPS, self._config.fps)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        if not cap.isOpened():
            cap.release()
            log.error("Failed to open V4L2 camera '%s' (%s -> %s)", self.name, self._config.device, device)
            return False
        if not self._apply_capture_controls(cap, cv2):
            cap.release()
            return False
        self._capture = cap
        self._running = True
        self._thread = threading.Thread(target=self._capture_loop, daemon=True, name=f"cam-{self.name}")
        self._thread.start()
        log.info("V4L2 '%s' (%s) started at %dx%d@%dfps", self.name, device,
                 self._config.width, self._config.height, self._config.fps)
        return True

    def _apply_capture_controls(self, cap, cv2) -> bool:
        """Apply configured UVC controls and fail rather than silently drifting."""
        if self._config.auto_exposure is not None:
            # V4L2 UVC menu: 1=manual, 3=aperture-priority automatic.
            mode = 3.0 if self._config.auto_exposure else 1.0
            if not cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, mode):
                log.error("V4L2 '%s' failed to set auto_exposure=%s", self.name, self._config.auto_exposure)
                return False
            actual_mode = cap.get(cv2.CAP_PROP_AUTO_EXPOSURE)
            if abs(actual_mode - mode) > 0.5:
                log.error("V4L2 '%s' auto_exposure readback %.1f != %.1f", self.name, actual_mode, mode)
                return False
        if self._config.exposure_time_absolute is not None:
            value = int(self._config.exposure_time_absolute)
            if not cap.set(cv2.CAP_PROP_EXPOSURE, float(value)):
                log.error("V4L2 '%s' failed to set exposure_time_absolute=%d", self.name, value)
                return False
            actual_value = cap.get(cv2.CAP_PROP_EXPOSURE)
            if abs(actual_value - value) > 0.5:
                log.error("V4L2 '%s' exposure readback %.1f != %d", self.name, actual_value, value)
                return False
        if self._config.auto_exposure is not None or self._config.exposure_time_absolute is not None:
            log.info(
                "V4L2 '%s' exposure configured: auto=%s absolute=%s",
                self.name,
                self._config.auto_exposure,
                self._config.exposure_time_absolute,
            )
        return True

    def stop(self):
        self._running = False
        # Release first so a blocking V4L2 read wakes before the thread join.
        capture = self._capture
        self._capture = None
        if capture is not None:
            capture.release()
        if self._thread is not None:
            self._thread.join(timeout=3.0)

    def get_frame(self) -> Optional[np.ndarray]:
        with self._lock:
            return self._frame.copy() if self._frame is not None else None

    def get_intrinsics(self) -> dict:
        supplied = self._config.intrinsics or {}
        return {
            "backend": "v4l2",
            "device": self._config.device,
            "resolved_device": os.path.realpath(self._config.device),
            "width": self._config.width,
            "height": self._config.height,
            "fps": self._config.fps,
            "auto_exposure": self._config.auto_exposure,
            "exposure_time_absolute": self._config.exposure_time_absolute,
            "calibration_status": "configured" if supplied else "uncalibrated",
            "camera_matrix": supplied.get("camera_matrix"),
            "distortion_model": supplied.get("distortion_model"),
            "distortion_coefficients": supplied.get("distortion_coefficients"),
        }

    def _capture_loop(self):
        import cv2
        capture = self._capture
        while self._running:
            ok, frame = capture.read()
            if not ok:
                time.sleep(0.02)
                continue
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            with self._lock:
                self._frame = frame
            self._frame_count += 1


class FFmpegV4L2Stream:
    """UVC capture isolated in its own FFmpeg process.

    OpenCV serializes or throttles concurrent V4L2 MJPEG decoding on this host.
    One FFmpeg process per camera keeps all four visual-tactile streams at their
    requested frame rate and also isolates driver/decoder failures.
    """

    _FRAME_TIMEOUT_S = 2.0
    _START_TIMEOUT_S = 5.0
    _RESTART_DELAY_S = 0.25

    def __init__(self, config: CameraConfig):
        self._config = config
        self._frame: Optional[np.ndarray] = None
        self._lock = threading.Lock()
        self._process_lock = threading.Lock()
        self._process: Optional[subprocess.Popen] = None
        self._running = False
        self._stop_event = threading.Event()
        self._first_frame = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._frame_count = 0
        self._last_ffmpeg_error = ""
        self._ffmpeg = ""
        self._v4l2_ctl = ""

    @property
    def name(self) -> str:
        return self._config.name

    @property
    def frame_count(self) -> int:
        return self._frame_count

    def start(self) -> bool:
        if self._running:
            return True

        device = os.path.realpath(self._config.device)
        if not self._config.device or not os.path.exists(device):
            log.error(
                "FFmpeg V4L2 camera '%s' device is missing (%s -> %s)",
                self.name,
                self._config.device,
                device,
            )
            return False

        self._ffmpeg = shutil.which("ffmpeg") or ""
        if not self._ffmpeg:
            log.error("FFmpeg is required for camera '%s' but was not found", self.name)
            return False
        if self._has_configured_controls():
            self._v4l2_ctl = shutil.which("v4l2-ctl") or ""
            if not self._v4l2_ctl:
                log.error(
                    "v4l2-ctl is required to configure exposure for camera '%s'",
                    self.name,
                )
                return False

        with self._lock:
            self._frame = None
        self._frame_count = 0
        self._last_ffmpeg_error = ""
        self._first_frame.clear()
        self._stop_event.clear()
        self._running = True
        self._thread = threading.Thread(
            target=self._capture_loop,
            daemon=True,
            name=f"ffmpeg-cam-{self.name}",
        )
        self._thread.start()

        if not self._first_frame.wait(timeout=self._START_TIMEOUT_S):
            detail = f": {self._last_ffmpeg_error}" if self._last_ffmpeg_error else ""
            log.error("FFmpeg V4L2 '%s' produced no frame within %.1fs%s",
                      self.name, self._START_TIMEOUT_S, detail)
            self.stop()
            return False

        log.info(
            "FFmpeg V4L2 '%s' (%s) started at %dx%d@%dfps",
            self.name,
            os.path.realpath(self._config.device),
            self._config.width,
            self._config.height,
            self._config.fps,
        )
        return True

    def stop(self):
        self._running = False
        self._stop_event.set()
        process = self._get_process()
        if process is not None and process.poll() is None:
            try:
                process.terminate()
            except OSError:
                pass

        thread = self._thread
        if thread is not None and thread is not threading.current_thread():
            thread.join(timeout=3.0)
            if thread.is_alive():
                process = self._get_process()
                if process is not None and process.poll() is None:
                    try:
                        process.kill()
                    except OSError:
                        pass
                thread.join(timeout=2.0)
                if thread.is_alive():
                    log.error("FFmpeg capture thread '%s' did not stop", self.name)
        self._thread = None

    def get_frame(self) -> Optional[np.ndarray]:
        with self._lock:
            return self._frame.copy() if self._frame is not None else None

    def get_intrinsics(self) -> dict:
        supplied = self._config.intrinsics or {}
        return {
            "backend": "ffmpeg_v4l2",
            "device": self._config.device,
            "resolved_device": os.path.realpath(self._config.device),
            "width": self._config.width,
            "height": self._config.height,
            "fps": self._config.fps,
            "auto_exposure": self._config.auto_exposure,
            "exposure_time_absolute": self._config.exposure_time_absolute,
            "calibration_status": "configured" if supplied else "uncalibrated",
            "camera_matrix": supplied.get("camera_matrix"),
            "distortion_model": supplied.get("distortion_model"),
            "distortion_coefficients": supplied.get("distortion_coefficients"),
        }

    def _has_configured_controls(self) -> bool:
        return (
            self._config.auto_exposure is not None
            or self._config.exposure_time_absolute is not None
        )

    def _apply_v4l2_controls(self, device: str) -> bool:
        controls: list[tuple[str, int]] = []
        if self._config.auto_exposure is not None:
            controls.append(("auto_exposure", 3 if self._config.auto_exposure else 1))
        if self._config.exposure_time_absolute is not None:
            controls.append(
                ("exposure_time_absolute", int(self._config.exposure_time_absolute))
            )
        if not controls:
            return True

        for control, expected in controls:
            try:
                result = subprocess.run(
                    [
                        self._v4l2_ctl,
                        "--device",
                        device,
                        f"--set-ctrl={control}={expected}",
                    ],
                    check=True,
                    capture_output=True,
                    text=True,
                    timeout=3.0,
                )
            except (OSError, subprocess.SubprocessError) as exc:
                stderr = getattr(exc, "stderr", "") or ""
                detail = stderr.strip() or str(exc)
                self._last_ffmpeg_error = detail
                log.error("V4L2 '%s' failed to set %s=%d: %s",
                          self.name, control, expected, detail)
                return False

            if result.stderr.strip():
                log.debug("v4l2-ctl '%s': %s", self.name, result.stderr.strip())

            try:
                result = subprocess.run(
                    [self._v4l2_ctl, "--device", device, f"--get-ctrl={control}"],
                    check=True,
                    capture_output=True,
                    text=True,
                    timeout=3.0,
                )
            except (OSError, subprocess.SubprocessError) as exc:
                stderr = getattr(exc, "stderr", "") or ""
                detail = stderr.strip() or str(exc)
                self._last_ffmpeg_error = detail
                log.error("V4L2 '%s' failed to read back %s: %s",
                          self.name, control, detail)
                return False

            match = re.search(rf"\b{re.escape(control)}\s*:\s*(-?\d+)", result.stdout)
            if match is None or int(match.group(1)) != expected:
                actual = match.group(1) if match is not None else result.stdout.strip()
                self._last_ffmpeg_error = (
                    f"{control} readback {actual!r}, expected {expected}"
                )
                log.error("V4L2 '%s' %s", self.name, self._last_ffmpeg_error)
                return False

        log.info(
            "V4L2 '%s' exposure configured: auto=%s absolute=%s",
            self.name,
            self._config.auto_exposure,
            self._config.exposure_time_absolute,
        )
        return True

    def _capture_loop(self):
        while self._running and not self._stop_event.is_set():
            device = os.path.realpath(self._config.device)
            failure = ""
            process: Optional[subprocess.Popen] = None
            stderr_thread: Optional[threading.Thread] = None

            if not os.path.exists(device):
                failure = f"device is missing: {self._config.device}"
            elif not self._apply_v4l2_controls(device):
                failure = self._last_ffmpeg_error or "failed to configure V4L2 controls"
            else:
                try:
                    self._last_ffmpeg_error = ""
                    process = subprocess.Popen(
                        self._ffmpeg_command(device),
                        stdin=subprocess.DEVNULL,
                        stdout=subprocess.PIPE,
                        stderr=subprocess.PIPE,
                        bufsize=0,
                    )
                    with self._process_lock:
                        self._process = process
                    stderr_thread = threading.Thread(
                        target=self._drain_stderr,
                        args=(process,),
                        daemon=True,
                        name=f"ffmpeg-stderr-{self.name}",
                    )
                    stderr_thread.start()
                    self._read_frames(process)
                    return_code = process.poll()
                    failure = self._last_ffmpeg_error or (
                        "FFmpeg output ended"
                        if return_code is None
                        else f"FFmpeg exited with status {return_code}"
                    )
                except (OSError, ValueError) as exc:
                    failure = str(exc)
                finally:
                    if process is not None:
                        self._reap_process(process)
                    if stderr_thread is not None:
                        stderr_thread.join(timeout=0.5)

            if self._running and not self._stop_event.is_set():
                with self._lock:
                    self._frame = None
                self._last_ffmpeg_error = failure
                log.warning(
                    "FFmpeg V4L2 '%s' stream stopped (%s); retrying in %.2fs",
                    self.name,
                    failure or "unknown reason",
                    self._RESTART_DELAY_S,
                )
                self._stop_event.wait(self._RESTART_DELAY_S)

    def _ffmpeg_command(self, device: str) -> list[str]:
        return [
            self._ffmpeg,
            "-hide_banner",
            "-nostdin",
            "-loglevel",
            "error",
            "-f",
            "v4l2",
            "-input_format",
            "mjpeg",
            "-video_size",
            f"{self._config.width}x{self._config.height}",
            "-framerate",
            str(self._config.fps),
            "-i",
            device,
            "-an",
            "-sn",
            "-dn",
            "-c:v",
            "rawvideo",
            "-pix_fmt",
            "rgb24",
            "-threads",
            "1",
            "-f",
            "rawvideo",
            "pipe:1",
        ]

    def _read_frames(self, process: subprocess.Popen) -> None:
        if process.stdout is None:
            raise ValueError("FFmpeg stdout pipe was not created")
        frame_size = self._config.width * self._config.height * 3
        fd = process.stdout.fileno()
        pending = bytearray()
        deadline = time.monotonic() + self._FRAME_TIMEOUT_S

        while self._running and not self._stop_event.is_set():
            ready, _, _ = select.select([fd], [], [], 0.2)
            if not ready:
                if process.poll() is not None:
                    return
                if time.monotonic() >= deadline:
                    self._last_ffmpeg_error = (
                        f"no complete frame for {self._FRAME_TIMEOUT_S:.1f}s"
                    )
                    return
                continue

            chunk = os.read(fd, frame_size - len(pending))
            if not chunk:
                return
            pending.extend(chunk)
            if len(pending) != frame_size:
                continue

            frame = np.frombuffer(pending, dtype=np.uint8).reshape(
                self._config.height,
                self._config.width,
                3,
            ).copy()
            with self._lock:
                self._frame = frame
            self._frame_count += 1
            self._first_frame.set()
            pending.clear()
            deadline = time.monotonic() + self._FRAME_TIMEOUT_S

    def _drain_stderr(self, process: subprocess.Popen) -> None:
        if process.stderr is None:
            return
        for raw_line in iter(process.stderr.readline, b""):
            line = raw_line.decode("utf-8", errors="replace").strip()
            if line:
                self._last_ffmpeg_error = line

    def _get_process(self) -> Optional[subprocess.Popen]:
        with self._process_lock:
            return self._process

    def _reap_process(self, process: subprocess.Popen) -> None:
        if process.poll() is None:
            try:
                process.terminate()
            except OSError:
                pass
        try:
            process.wait(timeout=1.0)
        except subprocess.TimeoutExpired:
            try:
                process.kill()
            except OSError:
                pass
            try:
                process.wait(timeout=1.0)
            except subprocess.TimeoutExpired:
                log.error("FFmpeg process for '%s' could not be reaped", self.name)
        if process.stdout is not None:
            process.stdout.close()
        if process.stderr is not None:
            process.stderr.close()
        with self._process_lock:
            if self._process is process:
                self._process = None


class MultiCameraCapture:
    """Manages RealSense and UVC camera streams."""

    def __init__(self, configs: list[CameraConfig]):
        self._streams = [self._make_stream(cfg) for cfg in configs]

    @staticmethod
    def _make_stream(config: CameraConfig):
        backend = config.backend.lower()
        if backend == "ffmpeg_v4l2":
            return FFmpegV4L2Stream(config)
        if backend == "v4l2":
            return V4L2Stream(config)
        if backend == "realsense":
            return RealSenseStream(config)
        raise ValueError(f"Unsupported camera backend: {config.backend}")

    def start_all(self) -> dict[str, bool]:
        results = {}
        for stream in self._streams:
            ok = stream.start()
            results[stream.name] = ok
        return results

    def stop_all(self):
        for stream in self._streams:
            stream.stop()

    def get_frames(self) -> dict[str, Optional[np.ndarray]]:
        return {s.name: s.get_frame() for s in self._streams}

    def get_frame(self, name: str) -> Optional[np.ndarray]:
        for s in self._streams:
            if s.name == name:
                return s.get_frame()
        return None

    def get_intrinsics(self) -> dict[str, dict]:
        return {s.name: s.get_intrinsics() for s in self._streams}

    @property
    def camera_names(self) -> list[str]:
        return [s.name for s in self._streams]


def list_realsense_devices() -> list[dict]:
    """Utility: list all connected RealSense devices with serial numbers."""
    import pyrealsense2 as rs
    ctx = rs.context()
    result = []
    for dev in ctx.query_devices():
        info = {
            "name": dev.get_info(rs.camera_info.name),
            "serial_number": dev.get_info(rs.camera_info.serial_number),
        }
        if dev.supports(rs.camera_info.usb_type_descriptor):
            info["usb_type"] = dev.get_info(rs.camera_info.usb_type_descriptor)
        result.append(info)
    return result
