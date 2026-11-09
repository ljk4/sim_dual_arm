"""Independent EGL renderer thread fed only by immutable captures.

The physics owner takes one complete ``rendering.Snapshot`` plus its frozen
measurement/identity at capture time.  This thread owns its own
``SceneRenderer``/``MjData``/EGL context, applies that snapshot and renders all
declared cameras once.  It never sees or mutates live physics data.
"""

from __future__ import annotations

import queue
import threading
import time
from collections.abc import Callable, Mapping
from dataclasses import dataclass
from types import MappingProxyType
from typing import Protocol

import numpy as np
from robot_contracts.models import RunContext

from .rendering import SceneRenderer, SceneRenderError, Snapshot
from .visibility import count_visible_pixels

__all__ = [
    "Capture",
    "RenderCloseSummary",
    "RenderedCapture",
    "RendererThread",
    "RenderFailure",
]

_BOUND_MESSAGE = "capture queue is bounded; only the oldest unrendered capture is dropped"


@dataclass(frozen=True, slots=True)
class Capture:
    """One coherent owner capture: physics state, measurement and identity."""

    context: RunContext
    capture_seq: int
    sim_time_ns: int
    joint_names: tuple[str, ...]
    positions: tuple[float, ...]
    velocities: tuple[float, ...]
    phase: str
    command_id: str | None
    snapshot: Snapshot

    def __post_init__(self) -> None:
        if not isinstance(self.context, RunContext):
            raise ValueError("capture.context must be a RunContext")
        for name in ("capture_seq", "sim_time_ns"):
            value = getattr(self, name)
            if isinstance(value, bool) or not isinstance(value, int) or value < 0:
                raise ValueError(f"capture.{name} must be a non-negative integer")
        if len(self.joint_names) != len(self.positions) or len(self.positions) != len(
            self.velocities
        ):
            raise ValueError("capture joint/position/velocity lengths differ")
        if not isinstance(self.snapshot, Snapshot):
            raise ValueError("capture.snapshot must be a rendering Snapshot")
        rounded = round(float(self.snapshot.sim_time) * 1_000_000_000)
        if rounded != self.sim_time_ns:
            raise ValueError("capture.sim_time_ns does not match the captured snapshot")


@dataclass(frozen=True, slots=True)
class RenderedCapture:
    """All three rendered views plus the exact capture that produced them."""

    capture: Capture
    frames: Mapping[str, np.ndarray]
    rendered_wall_ns: int

    quality: Mapping[str, Mapping[str, int]] | None = None

    def __post_init__(self) -> None:
        if not isinstance(self.capture, Capture):
            raise ValueError("rendered.capture must be a Capture")
        if not isinstance(self.frames, Mapping) or not self.frames:
            raise ValueError("rendered.frames must be a non-empty mapping")
        if isinstance(self.rendered_wall_ns, bool) or not isinstance(
            self.rendered_wall_ns, int
        ):
            raise ValueError("rendered.rendered_wall_ns must be an integer")
        if self.quality is not None:
            for camera, counts in self.quality.items():
                if not isinstance(camera, str) or not isinstance(counts, Mapping):
                    raise ValueError("rendered.quality must be camera -> object counts")
                for name, value in counts.items():
                    if not isinstance(name, str) or isinstance(value, bool) or value < 0:
                        raise ValueError("rendered quality counts must be non-negative")


@dataclass(frozen=True, slots=True)
class RenderFailure:
    reason: str
    detail: str


@dataclass(frozen=True, slots=True)
class RenderCloseSummary:
    closed: bool
    reason: str
    dropped_snapshots: int
    dropped_outputs: int
    rendered: int
    failure: RenderFailure | None


class _Renderer(Protocol):
    def render_rgb(self, snapshot: Snapshot) -> Mapping[str, np.ndarray]: ...

    def close(self) -> None: ...


RendererFactory = Callable[[object, tuple[str, ...], int, int], _Renderer]


def _default_factory(
    model: object, camera_names: tuple[str, ...], width: int, height: int
) -> _Renderer:
    return SceneRenderer(model, camera_names, width=width, height=height)


class RendererThread:
    """One bounded renderer thread with a private EGL context."""

    def __init__(
        self,
        model: object,
        camera_names: tuple[str, ...],
        *,
        width: int,
        height: int,
        snapshot_capacity: int,
        output_capacity: int,
        health_timeout_s: float,
        renderer_factory: RendererFactory | None = None,
        segmentation_geoms: Mapping[str, tuple[int, int]] | None = None,
    ) -> None:
        if not camera_names:
            raise ValueError("camera_names must not be empty")
        if snapshot_capacity < 1 or output_capacity < 1:
            raise ValueError("renderer queue capacities must be positive")
        if not (health_timeout_s > 0.0):
            raise ValueError("renderer health timeout must be positive")
        self._model = model
        self._camera_names = tuple(camera_names)
        self._width = int(width)
        self._height = int(height)
        self._factory = _default_factory if renderer_factory is None else renderer_factory
        self._segmentation_geoms = (
            None if segmentation_geoms is None else dict(segmentation_geoms)
        )
        self._snapshots: queue.Queue[Capture] = queue.Queue(maxsize=snapshot_capacity)
        self._outputs: queue.Queue[RenderedCapture] = queue.Queue(maxsize=output_capacity)
        self._health_timeout_ns = round(float(health_timeout_s) * 1_000_000_000)
        self._lock = threading.Lock()
        self._ready = threading.Event()
        self._stop = threading.Event()
        self._failure: RenderFailure | None = None
        self._startup_error: BaseException | None = None
        self._last_progress_ns = time.monotonic_ns()
        self._started_ns = self._last_progress_ns
        self._dropped_snapshots = 0
        self._dropped_outputs = 0
        self._rendered = 0
        self._in_flight = False
        self._closed = False
        self._close_summary: RenderCloseSummary | None = None
        self._thread = threading.Thread(
            target=self._run,
            name="backend-renderer",
            daemon=True,
        )

    def start(self) -> None:
        self._thread.start()

    def wait_ready(self, timeout_s: float) -> None:
        if not self._ready.wait(timeout_s):
            raise TimeoutError("renderer did not become ready")
        if self._startup_error is not None:
            raise self._startup_error

    @property
    def camera_names(self) -> tuple[str, ...]:
        return self._camera_names

    @property
    def failure(self) -> RenderFailure | None:
        with self._lock:
            return self._failure

    @property
    def last_progress_ns(self) -> int:
        with self._lock:
            return self._last_progress_ns

    @property
    def health_timeout_ns(self) -> int:
        return self._health_timeout_ns

    @property
    def dropped_snapshots(self) -> int:
        with self._lock:
            return self._dropped_snapshots

    @property
    def dropped_outputs(self) -> int:
        with self._lock:
            return self._dropped_outputs

    @property
    def rendered_count(self) -> int:
        with self._lock:
            return self._rendered

    def submit(self, capture: Capture) -> bool:
        """Queue one capture, dropping only an older *unrendered* capture if full."""
        with self._lock:
            if self._closed or self._failure is not None:
                return False
        try:
            self._snapshots.put_nowait(capture)
            return True
        except queue.Full:
            pass
        with self._lock:
            self._dropped_snapshots += 1
        try:
            self._snapshots.get_nowait()
        except queue.Empty:  # pragma: no cover - consumer took it between calls
            pass
        try:
            self._snapshots.put_nowait(capture)
        except queue.Full:  # pragma: no cover - one consumer, bounded race
            with self._lock:
                self._dropped_snapshots += 1
            return False
        return True

    def drain(self, limit: int = 64) -> tuple[RenderedCapture, ...]:
        items: list[RenderedCapture] = []
        for _ in range(limit):
            try:
                items.append(self._outputs.get_nowait())
            except queue.Empty:
                break
        return tuple(items)

    def check_health(self, wall_ns: int | None = None) -> RenderFailure | None:
        failure = self.failure
        if failure is not None:
            return failure
        now = time.monotonic_ns() if wall_ns is None else wall_ns
        with self._lock:
            # Only a non-empty input or in-flight render can be stalled; an
            # idle renderer with no work is healthy by definition.
            busy = self._snapshots.qsize() > 0 or self._in_flight
            last = self._last_progress_ns
        if busy and now - last > self._health_timeout_ns:
            return RenderFailure(
                "RENDER_STALLED",
                f"renderer made no progress for {(now - last) / 1e9:.3f}s",
            )
        return None

    def request_failure(self, reason: str, detail: str = "") -> None:
        """Record a failure detected outside the renderer (bounded observation)."""
        with self._lock:
            if self._failure is None:
                self._failure = RenderFailure(reason, detail or reason)

    def close(self, timeout_s: float) -> RenderCloseSummary:
        """Stop and close the EGL context on the renderer's own thread."""
        with self._lock:
            if self._close_summary is not None:
                return self._close_summary
            self._closed = True
        self._stop.set()
        self._thread.join(timeout=max(0.0, timeout_s))
        with self._lock:
            if self._close_summary is None:
                if self._thread.is_alive():
                    self._failure = RenderFailure(
                        "RENDER_CLOSE_TIMEOUT", "renderer thread did not stop within budget"
                    )
                else:  # pragma: no cover - normal path sets it in _run
                    self._failure = RenderFailure(
                        "RENDER_CLOSE_FAILED", "renderer exited without a close summary"
                    )
                self._close_summary = RenderCloseSummary(
                    closed=False,
                    reason="RENDER_CLOSE_TIMEOUT",
                    dropped_snapshots=self._dropped_snapshots,
                    dropped_outputs=self._dropped_outputs,
                    rendered=self._rendered,
                    failure=self._failure,
                )
            return self._close_summary

    def _run(self) -> None:
        renderer: _Renderer | None = None
        startup_error: BaseException | None = None
        try:
            renderer = self._factory(
                self._model, self._camera_names, self._width, self._height
            )
        except BaseException as error:  # noqa: BLE001 - startup fault is reported
            startup_error = error
        with self._lock:
            self._startup_error = startup_error
            if startup_error is not None:
                self._failure = RenderFailure(
                    "RENDER_START_FAILED", f"{type(startup_error).__name__}: {startup_error}"
                )
            self._last_progress_ns = time.monotonic_ns()
        self._ready.set()
        if startup_error is not None:
            return

        close_error: BaseException | None = None
        try:
            while True:
                try:
                    capture = self._snapshots.get(timeout=0.05)
                except queue.Empty:
                    if self._stop.is_set():
                        break
                    continue
                with self._lock:
                    self._last_progress_ns = time.monotonic_ns()
                    self._in_flight = True
                try:
                    assert renderer is not None
                    frames = renderer.render_rgb(capture.snapshot)
                    quality = self._render_quality(renderer, capture)
                    result = RenderedCapture(
                        capture=capture,
                        frames=MappingProxyType(dict(frames)),
                        rendered_wall_ns=time.monotonic_ns(),
                        quality=quality,
                    )
                except BaseException as error:  # noqa: BLE001 - fault must stop the backend
                    reason = (
                        error.reason
                        if isinstance(error, SceneRenderError)
                        else "RENDER_FAILED"
                    )
                    with self._lock:
                        self._failure = RenderFailure(
                            reason, f"{type(error).__name__}: {error}"
                        )
                        self._last_progress_ns = time.monotonic_ns()
                        self._in_flight = False
                    break
                self._publish(result)
                with self._lock:
                    self._rendered += 1
                    self._last_progress_ns = time.monotonic_ns()
                    self._in_flight = False
        finally:
            if renderer is not None:
                try:
                    renderer.close()
                except BaseException as error:  # noqa: BLE001 - report, then exit
                    close_error = error
            with self._lock:
                failure = self._failure
                if close_error is not None and failure is None:
                    failure = RenderFailure(
                        "RENDER_CLOSE_FAILED", f"{type(close_error).__name__}: {close_error}"
                    )
                    self._failure = failure
                reason = "CLOSED" if failure is None else failure.reason
                self._close_summary = RenderCloseSummary(
                    closed=failure is None,
                    reason=reason,
                    dropped_snapshots=self._dropped_snapshots,
                    dropped_outputs=self._dropped_outputs,
                    rendered=self._rendered,
                    failure=failure,
                )

    def _render_quality(
        self, renderer: _Renderer, capture: Capture
    ) -> Mapping[str, Mapping[str, int]] | None:
        targets = self._segmentation_geoms
        if not targets:
            return None
        render = getattr(renderer, "render_segmentation", None)
        if render is None:
            return None
        segmentations = render(capture.snapshot)
        counts: dict[str, dict[str, int]] = {}
        for camera, segmentation in segmentations.items():
            counts[camera] = {
                name: count_visible_pixels(
                    segmentation, object_id, object_type=object_type
                )
                for name, (object_id, object_type) in targets.items()
            }
        return MappingProxyType(
            {camera: MappingProxyType(values) for camera, values in counts.items()}
        )

    def _publish(self, result: RenderedCapture) -> None:
        try:
            self._outputs.put_nowait(result)
            return
        except queue.Full:
            pass
        with self._lock:
            self._dropped_outputs += 1
        try:
            self._outputs.get_nowait()
        except queue.Empty:  # pragma: no cover - single consumer
            pass
        try:
            self._outputs.put_nowait(result)
        except queue.Full:  # pragma: no cover - one consumer, bounded race
            with self._lock:
                self._dropped_outputs += 1
