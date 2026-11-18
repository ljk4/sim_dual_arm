"""Private EGL rendering data and immutable scene snapshots.

The renderer owns one MjData that is never the physics Scene.data.  A snapshot
is an independent copy of MuJoCo's native ``mjSTATE_FULLPHYSICS`` vector plus
the simulation time.  Applying a snapshot uses ``mj_setState``/``mj_forward``
on that private data, so later physics changes cannot alter earlier images and
rendering cannot write the physics data.
"""

from __future__ import annotations

from collections.abc import Mapping, Sequence
from dataclasses import dataclass
from types import MappingProxyType

import mujoco
import numpy as np

__all__ = [
    "GLInfo",
    "SceneRenderer",
    "SceneRenderError",
    "Snapshot",
    "apply_snapshot",
    "capture_snapshot",
    "derive_snapshot",
    "read_gl_info",
]

STATE_SIGNATURE = mujoco.mjtState.mjSTATE_FULLPHYSICS


class SceneRenderError(RuntimeError):
    """Rendering failed with a stable reason and no partially usable output."""

    def __init__(self, reason: str, message: str = "") -> None:
        super().__init__(message or reason)
        self.reason = reason
        self.message = message or reason


@dataclass(frozen=True, slots=True)
class GLInfo:
    vendor: str
    renderer: str
    version: str
    nvidia: bool


@dataclass(frozen=True, slots=True)
class Snapshot:
    """One independent, read-only full-physics snapshot."""

    state: np.ndarray
    sim_time: float

    def __post_init__(self) -> None:
        state = np.array(self.state, dtype=np.float64, copy=True)
        if state.ndim != 1 or state.size == 0:
            raise ValueError("snapshot state must be a non-empty one-dimensional array")
        if not np.isfinite(state).all():
            raise ValueError("snapshot state must contain only finite values")
        state.flags.writeable = False
        object.__setattr__(self, "state", state)
        sim_time = float(self.sim_time)
        if not np.isfinite(sim_time):
            raise ValueError("snapshot sim_time must be finite")
        reported_time = float(state[0])
        if sim_time != reported_time:
            raise ValueError(
                "snapshot sim_time does not match the native mjSTATE_FULLPHYSICS "
                f"TIME element: {sim_time!r} != {reported_time!r}"
            )
        object.__setattr__(self, "sim_time", sim_time)


def _state_size(model: mujoco.MjModel) -> int:
    return int(mujoco.mj_stateSize(model, STATE_SIGNATURE))


def capture_snapshot(model: mujoco.MjModel, data: mujoco.MjData) -> Snapshot:
    """Copy the full physics state out of one MjData without retaining it."""
    size = _state_size(model)
    state = np.empty(size, dtype=np.float64)
    mujoco.mj_getState(model, data, state, STATE_SIGNATURE)
    return Snapshot(state=state, sim_time=float(data.time))


def _check_snapshot(model: mujoco.MjModel, snapshot: Snapshot) -> None:
    if not isinstance(snapshot, Snapshot):
        raise SceneRenderError("snapshot_invalid", "snapshot must be a Snapshot")
    expected = _state_size(model)
    if snapshot.state.size != expected:
        raise SceneRenderError(
            "snapshot_size_mismatch",
            f"snapshot has {snapshot.state.size} values, model expects {expected}",
        )


def _restore(model: mujoco.MjModel, data: mujoco.MjData, snapshot: Snapshot) -> None:
    _check_snapshot(model, snapshot)
    try:
        mujoco.mj_setState(model, data, snapshot.state, STATE_SIGNATURE)
    except Exception as error:
        raise SceneRenderError(
            "snapshot_apply_failed", f"{type(error).__name__}: {error}"
        ) from error


def apply_snapshot(model: mujoco.MjModel, data: mujoco.MjData, snapshot: Snapshot) -> None:
    """Restore a snapshot into caller-owned data and recompute derived state."""
    _restore(model, data, snapshot)
    try:
        mujoco.mj_forward(model, data)
    except Exception as error:
        raise SceneRenderError(
            "snapshot_forward_failed", f"{type(error).__name__}: {error}"
        ) from error


def derive_snapshot(
    model: mujoco.MjModel,
    base: Snapshot,
    qpos_updates: Mapping[int, Sequence[float]],
) -> Snapshot:
    """Return a new snapshot with explicit qpos slices applied to a private MjData."""
    _check_snapshot(model, base)
    updates: list[tuple[int, tuple[float, ...]]] = []
    for address, values in qpos_updates.items():
        if isinstance(address, bool) or not isinstance(address, int):
            raise SceneRenderError("qpos_update_invalid", "qpos addresses must be integers")
        numbers = tuple(float(value) for value in values)
        if not numbers:
            raise SceneRenderError("qpos_update_invalid", "qpos updates must not be empty")
        if address < 0 or address + len(numbers) > model.nq:
            raise SceneRenderError(
                "qpos_update_invalid",
                f"qpos update [{address}, {address + len(numbers)}) is outside [0, {model.nq})",
            )
        if not np.isfinite(numbers).all():
            raise SceneRenderError("qpos_update_invalid", "qpos updates must be finite")
        updates.append((address, numbers))
    updates.sort()
    for (first_address, first_values), (second_address, _second_values) in zip(
        updates, updates[1:], strict=False
    ):
        if first_address + len(first_values) > second_address:
            raise SceneRenderError("qpos_update_invalid", "qpos updates overlap")
    data = mujoco.MjData(model)
    _restore(model, data, base)
    for address, numbers in updates:
        data.qpos[address : address + len(numbers)] = numbers
    try:
        mujoco.mj_forward(model, data)
    except Exception as error:
        raise SceneRenderError(
            "snapshot_forward_failed", f"{type(error).__name__}: {error}"
        ) from error
    return capture_snapshot(model, data)


def _gl_string(name: int) -> str | None:
    from OpenGL import GL

    value = GL.glGetString(name)
    if value is None:
        return None
    if isinstance(value, bytes):
        return value.decode("utf-8", "replace")
    return str(value)


def read_gl_info() -> GLInfo:
    """Query the current EGL context and require the local NVIDIA driver."""
    try:
        vendor = _gl_string(0x1F00)
        renderer = _gl_string(0x1F01)
        version = _gl_string(0x1F02)
    except Exception as error:
        raise SceneRenderError("opengl_query_failed", f"{type(error).__name__}: {error}") from error
    if not vendor or not renderer or not version:
        raise SceneRenderError(
            "opengl_query_failed",
            "OpenGL did not return vendor/renderer/version strings",
        )
    lowered = f"{vendor} {renderer}".lower()
    software = any(token in lowered for token in ("llvmpipe", "softpipe", "swrast"))
    nvidia = "nvidia" in vendor.lower() and not software
    info = GLInfo(vendor=vendor, renderer=renderer, version=version, nvidia=nvidia)
    if not nvidia:
        raise SceneRenderError(
            "opengl_not_nvidia",
            f"OpenGL is not the local NVIDIA driver: vendor={vendor!r}, renderer={renderer!r}",
        )
    return info


def _freeze_rgb(image: object, camera: str, width: int, height: int) -> np.ndarray:
    array = np.asarray(image)
    if array.dtype != np.uint8 or array.shape != (height, width, 3):
        raise SceneRenderError(
            "render_frame_invalid",
            f"camera {camera!r} RGB frame is {array.dtype} {array.shape}, "
            f"expected uint8 {(height, width, 3)}",
        )
    frozen = np.array(array, copy=True)
    frozen.flags.writeable = False
    return frozen


def _freeze_segmentation(image: object, camera: str, width: int, height: int) -> np.ndarray:
    array = np.asarray(image)
    if array.dtype.kind not in ("i", "u"):
        raise SceneRenderError(
            "segmentation_frame_invalid",
            f"camera {camera!r} segmentation dtype is {array.dtype}, expected integer",
        )
    if array.shape != (height, width, 2):
        raise SceneRenderError(
            "segmentation_frame_invalid",
            f"camera {camera!r} segmentation is {array.shape}, expected {(height, width, 2)}",
        )
    frozen = np.array(array, copy=True)
    frozen.flags.writeable = False
    return frozen


class SceneRenderer:
    """One EGL renderer and its private MjData for named camera views."""

    def __init__(
        self,
        model: mujoco.MjModel,
        camera_names: Sequence[str],
        *,
        width: int,
        height: int,
    ) -> None:
        if isinstance(width, bool) or not isinstance(width, int) or width <= 0:
            raise SceneRenderError("render_size_invalid", "width must be a positive integer")
        if isinstance(height, bool) or not isinstance(height, int) or height <= 0:
            raise SceneRenderError("render_size_invalid", "height must be a positive integer")
        names = tuple(camera_names)
        if not names or len(set(names)) != len(names):
            raise SceneRenderError(
                "camera_names_invalid", "camera names must be non-empty and unique"
            )
        camera_ids: dict[str, int] = {}
        for name in names:
            try:
                camera_ids[name] = int(model.camera(name).id)
            except KeyError as error:
                raise SceneRenderError(
                    "camera_missing", f"camera {name!r} is not in the compiled model"
                ) from error
        offwidth = int(model.vis.global_.offwidth)
        offheight = int(model.vis.global_.offheight)
        if offwidth < width or offheight < height:
            raise SceneRenderError(
                "offscreen_too_small",
                f"offscreen {offwidth}x{offheight} is smaller than configured {width}x{height}",
            )
        for name, camera_id in camera_ids.items():
            resolution = tuple(int(value) for value in model.cam_resolution[camera_id])
            if resolution != (width, height):
                raise SceneRenderError(
                    "camera_resolution_mismatch",
                    f"camera {name!r} resolution is {resolution}, expected {(width, height)}",
                )
        self._model = model
        self._camera_ids = MappingProxyType(camera_ids)
        self._width = width
        self._height = height
        self._data = mujoco.MjData(model)
        self._renderer: mujoco.Renderer | None = None
        self._closed = False
        try:
            self._renderer = mujoco.Renderer(model, height=height, width=width)
            # Bind the EGL context and exercise the real render path once so
            # the GL vendor query is not a context-free guess.  The frame is
            # discarded; snapshot rendering below still owns every reported
            # view.
            mujoco.mj_forward(model, self._data)
            self._renderer.update_scene(self._data, camera=next(iter(camera_ids.values())))
            self._renderer.render()
            self._gl = read_gl_info()
        except BaseException:
            self._closed = True
            renderer = self._renderer
            self._renderer = None
            if renderer is not None:
                try:
                    renderer.close()
                except Exception:
                    pass
            raise

    @property
    def camera_names(self) -> tuple[str, ...]:
        return tuple(self._camera_ids)

    @property
    def width(self) -> int:
        return self._width

    @property
    def height(self) -> int:
        return self._height

    @property
    def gl_info(self) -> GLInfo:
        return self._gl

    def capture(self, data: mujoco.MjData) -> Snapshot:
        return capture_snapshot(self._model, data)

    def derive_snapshot(
        self, base: Snapshot, qpos_updates: Mapping[int, Sequence[float]]
    ) -> Snapshot:
        return derive_snapshot(self._model, base, qpos_updates)

    def render_rgb(self, snapshot: Snapshot) -> Mapping[str, np.ndarray]:
        return self._render(snapshot, segmentation=False)

    def render_segmentation(self, snapshot: Snapshot) -> Mapping[str, np.ndarray]:
        return self._render(snapshot, segmentation=True)

    def _render(self, snapshot: Snapshot, *, segmentation: bool) -> Mapping[str, np.ndarray]:
        if self._closed or self._renderer is None:
            raise SceneRenderError("renderer_closed", "render called after close")
        apply_snapshot(self._model, self._data, snapshot)
        views: dict[str, np.ndarray] = {}
        for name, camera_id in self._camera_ids.items():
            try:
                self._renderer.update_scene(self._data, camera=camera_id)
                if segmentation:
                    self._renderer.enable_segmentation_rendering()
                    try:
                        raw = self._renderer.render()
                    finally:
                        self._renderer.disable_segmentation_rendering()
                    views[name] = _freeze_segmentation(raw, name, self._width, self._height)
                else:
                    # Never inherit a segmentation flag from an earlier view.
                    self._renderer.disable_segmentation_rendering()
                    raw = self._renderer.render()
                    views[name] = _freeze_rgb(raw, name, self._width, self._height)
            except SceneRenderError:
                raise
            except Exception as error:
                raise SceneRenderError(
                    "render_failed",
                    f"camera {name!r} ({'segmentation' if segmentation else 'rgb'}): "
                    f"{type(error).__name__}: {error}",
                ) from error
        return MappingProxyType(views)

    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        renderer = self._renderer
        self._renderer = None
        if renderer is not None:
            try:
                renderer.close()
            except Exception as error:
                raise SceneRenderError(
                    "renderer_close_failed", f"{type(error).__name__}: {error}"
                ) from error
