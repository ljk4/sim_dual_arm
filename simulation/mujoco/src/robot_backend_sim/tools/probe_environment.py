"""MuJoCo/mink/EGL environment probe for one report run.

The module is intentionally small: it reports installed dependency versions and
import sources, loads the local single Panda keyframe,
runs a few real MuJoCo steps,
solves one known QP through DAQP, renders one 320x240 RGB preview with EGL and
publishes a JSON report.  It does not start ROS, a robot service or training.
"""

from __future__ import annotations

import argparse
import hashlib
import importlib
import importlib.metadata
import json
import math
import os
import platform
import re
import sys
import tempfile
import time
import uuid
from dataclasses import dataclass, field
from datetime import UTC, datetime
from pathlib import Path

from robot_backend_sim.common.physics import assets

COMPONENT = "sim_probe"
REPORT_SCHEMA_VERSION = 1
PREVIEW_WIDTH = 320
PREVIEW_HEIGHT = 240
MODEL_STEPS = 20
QP_TOLERANCE = 1e-6
_RUN_ID_PATTERN = re.compile(r"^[A-Za-z0-9][A-Za-z0-9._-]{0,127}$")

_IMPORT_MODULES = ("robot_backend_sim", "mujoco", "mink", "qpsolvers", "numpy", "PIL", "OpenGL")
_PACKAGE_DISTRIBUTIONS = (
    "mujoco",
    "mink",
    "numpy",
    "scipy",
    "qpsolvers",
    "pillow",
    "PyOpenGL",
    "ruckig",
    "absl-py",
    "daqp",
    "etils",
    "fsspec",
    "glfw",
    "typing_extensions",
    "websockets",
    "zipp",
)


class ProbeUsageError(ValueError):
    """Invalid probe invocation before the report run directory is created."""


class ProbeFailure(RuntimeError):
    """A stable environment-check failure with optional partial evidence."""

    def __init__(self, reason: str, message: str = "", *, details: dict | None = None) -> None:
        super().__init__(message or reason)
        self.reason = reason
        self.message = message or reason
        self.details = details or {}


@dataclass
class ProbeArguments:
    report_root: Path
    run_id: str


@dataclass
class ProbeState:
    report: dict[str, object] = field(default_factory=dict)
    failure: ProbeFailure | None = None
    log_close_error: str | None = None
    logger: object | None = None


def _utc_now() -> str:
    return datetime.now(UTC).isoformat(timespec="milliseconds").replace("+00:00", "Z")


def _validate_run_id(value: str) -> str:
    if not isinstance(value, str) or not _RUN_ID_PATTERN.fullmatch(value):
        raise ProbeUsageError(
            "run id must contain 1..128 ASCII letters, digits, '.', '_' or '-' "
            "and must start with a letter or digit"
        )
    if value in {".", ".."} or ".." in value:
        raise ProbeUsageError("run id must not be '.' or '..' or contain '..'")
    return value


def _validate_report_root(path: Path) -> Path:
    resolved = path.resolve()
    if resolved == Path("/"):
        raise ProbeUsageError("report root must not be the filesystem root")
    home = Path.home().resolve()
    if resolved == home or resolved in home.parents:
        raise ProbeUsageError("report root must not be the home directory or a parent")
    return resolved


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--report-root", type=Path, required=True, help="report parent directory")
    parser.add_argument(
        "--run-id",
        default=None,
        help="fresh report run identity (default: probe-<UTC>-<pid>)",
    )
    return parser


def parse_arguments(argv: list[str] | None = None) -> ProbeArguments:
    parsed = _parser().parse_args(argv)
    run_id = (
        f"probe-{time.strftime('%Y%m%dT%H%M%SZ', time.gmtime())}-{os.getpid()}"
        if parsed.run_id is None
        else parsed.run_id
    )
    return ProbeArguments(
        report_root=_validate_report_root(parsed.report_root),
        run_id=_validate_run_id(run_id),
    )


def _version(distribution: str) -> str | None:
    try:
        return importlib.metadata.version(distribution)
    except importlib.metadata.PackageNotFoundError:
        return None


def check_dependencies() -> dict[str, object]:
    installed = {name: _version(name) for name in _PACKAGE_DISTRIBUTIONS}
    modules: dict[str, str | None] = {}
    for module_name in _IMPORT_MODULES:
        try:
            module = importlib.import_module(module_name)
        except Exception as error:
            raise ProbeFailure(
                "dependency_import_failed",
                f"{module_name}: {type(error).__name__}: {error}",
                details={"packages": installed},
            ) from error
        filename = getattr(module, "__file__", None)
        modules[module_name] = str(Path(filename).resolve()) if filename else None
    sim_file = modules.get("robot_backend_sim")
    if not sim_file:
        raise ProbeFailure(
            "sim_module_missing",
            "robot_backend_sim has no importable file",
            details={"packages": installed},
        )
    module = sys.modules.get("robot_backend_sim")
    package_root = getattr(module, "__path__", None)
    if not package_root:
        raise ProbeFailure(
            "sim_module_missing",
            "robot_backend_sim has no package path",
            details={"packages": installed},
        )
    return {
        "python": {
            "implementation": platform.python_implementation(),
            "version": platform.python_version(),
            "executable": sys.executable,
        },
        "packages": installed,
        "imports": modules,
        "sim_package_root": str(Path(package_root[0]).resolve()),
    }


def _verify_mounted_source(dependency_info: dict[str, object]) -> None:
    expected = Path(
        os.environ.get("PROJECT_SOURCE_ROOT", Path(__file__).resolve().parents[1])
    ).resolve()
    actual = Path(str(dependency_info["sim_package_root"]))
    if actual != expected and expected not in actual.parents:
        raise ProbeFailure(
            "sim_source_shadowed",
            "robot_backend_sim imported from "
            f"{actual}, expected the mounted source under {expected}",
            details={"dependencies": dependency_info},
        )


def _model_stats(model: object) -> dict[str, int]:
    return {
        "nq": int(model.nq),
        "nv": int(model.nv),
        "nu": int(model.nu),
        "nkey": int(model.nkey),
        "nbody": int(model.nbody),
        "ngeom": int(model.ngeom),
        "nmesh": int(model.nmesh),
        "ncam": int(model.ncam),
    }


def _check_model() -> dict[str, object]:
    import mujoco
    import numpy as np

    from robot_backend_sim.panda_sim.plugin import panda_model_paths

    scene_path, panda_path = panda_model_paths()
    for path in (scene_path, panda_path):
        if not path.is_file():
            raise ProbeFailure("model_missing", f"model file is missing: {path}")
    try:
        model = mujoco.MjModel.from_xml_path(str(scene_path))
        panda_model = mujoco.MjModel.from_xml_path(str(panda_path))
    except Exception as error:
        raise ProbeFailure(
            "model_load_failed",
            f"{type(error).__name__}: {error}",
        ) from error
    stats = _model_stats(model)
    if stats["nq"] < 7 or stats["nv"] < 7 or stats["nu"] < 7:
        raise ProbeFailure("model_dimensions_implausible", f"model stats are too small: {stats}")
    if stats["nq"] > 32 or stats["nv"] > 32 or stats["nu"] > 32:
        raise ProbeFailure("model_dimensions_implausible", f"model stats are too large: {stats}")
    if stats["nkey"] < 1:
        raise ProbeFailure("keyframe_missing", "the upstream model has no keyframe")
    data = mujoco.MjData(model)
    data.qpos[:] = model.key_qpos[0]
    data.qvel[:] = 0.0
    if stats["nu"]:
        data.ctrl[:] = model.key_ctrl[0]
    mujoco.mj_forward(model, data)
    for _ in range(MODEL_STEPS):
        mujoco.mj_step(model, data)
    finite = {
        "qpos": bool(np.isfinite(data.qpos).all()),
        "qvel": bool(np.isfinite(data.qvel).all()),
        "ctrl": bool(np.isfinite(data.ctrl).all()),
    }
    if not all(finite.values()):
        raise ProbeFailure("model_state_not_finite", f"non-finite MuJoCo state: {finite}")
    return {
        "scene_xml": str(scene_path),
        "panda_xml": str(panda_path),
        "stats": stats,
        "panda_stats": _model_stats(panda_model),
        "steps": MODEL_STEPS,
        "finite": finite,
        "qpos": [float(value) for value in data.qpos],
        "qvel_max_abs": float(abs(data.qvel).max()) if data.qvel.size else 0.0,
    }


def _check_qp() -> dict[str, object]:
    import numpy as np
    import qpsolvers

    available = list(getattr(qpsolvers, "available_solvers", ()))
    if "daqp" not in available:
        raise ProbeFailure("daqp_unavailable", f"qpsolvers solvers: {available}")
    try:
        import daqp  # noqa: F401
    except Exception as error:
        raise ProbeFailure("daqp_unavailable", f"{type(error).__name__}: {error}") from error
    matrix = np.array([[2.0, 0.0], [0.0, 2.0]], dtype=float)
    linear = np.array([-2.0, -4.0], dtype=float)
    constraint = np.array([[1.0, 1.0]], dtype=float)
    bound = np.array([3.0], dtype=float)
    expected = np.array([1.0, 2.0], dtype=float)
    try:
        solution = qpsolvers.solve_qp(matrix, linear, A=constraint, b=bound, solver="daqp")
    except Exception as error:
        raise ProbeFailure("qp_solve_failed", f"{type(error).__name__}: {error}") from error
    if solution is None:
        raise ProbeFailure("qp_solve_failed", "DAQP returned no solution")
    values = np.asarray(solution, dtype=float).reshape(-1)
    error = float(np.max(np.abs(values - expected))) if values.size == 2 else math.inf
    if values.shape != (2,) or not math.isfinite(error) or error > QP_TOLERANCE:
        raise ProbeFailure(
            "qp_result_mismatch",
            f"expected {expected.tolist()}, got {values.tolist()}, error={error}",
        )
    return {
        "solver": "daqp",
        "available_solvers": available,
        "solution": [float(value) for value in values],
        "expected": [float(value) for value in expected],
        "max_abs_error": error,
        "tolerance": QP_TOLERANCE,
    }


def _gl_string(name: int) -> str | None:
    from OpenGL import GL

    value = GL.glGetString(name)
    if value is None:
        return None
    if isinstance(value, bytes):
        return value.decode("utf-8", "replace")
    return str(value)


def _prepare_render():
    import mujoco

    if os.environ.get("MUJOCO_GL") != "egl":
        raise ProbeFailure("egl_not_selected", "MUJOCO_GL must be 'egl'")
    if os.environ.get("PYOPENGL_PLATFORM") != "egl":
        raise ProbeFailure("egl_not_selected", "PYOPENGL_PLATFORM must be 'egl'")
    from robot_backend_sim.panda_sim.plugin import panda_model_paths

    scene_path, _panda_path = panda_model_paths()
    model = mujoco.MjModel.from_xml_path(str(scene_path))
    data = mujoco.MjData(model)
    data.qpos[:] = model.key_qpos[0]
    data.qvel[:] = 0.0
    if model.nu:
        data.ctrl[:] = model.key_ctrl[0]
    mujoco.mj_forward(model, data)
    camera = 0 if model.ncam else -1
    info: dict[str, object] = {
        "width": PREVIEW_WIDTH,
        "height": PREVIEW_HEIGHT,
        "camera": camera,
        "camera_count": int(model.ncam),
    }
    return model, data, camera, info


def _capture_render(model, data, camera, info):
    import mujoco
    import numpy as np

    renderer = mujoco.Renderer(model, height=PREVIEW_HEIGHT, width=PREVIEW_WIDTH)
    try:
        renderer.update_scene(data, camera=camera)
        frame = np.array(renderer.render(), copy=True)
        try:
            vendor = _gl_string(0x1F00)  # GL_VENDOR
            renderer_name = _gl_string(0x1F01)  # GL_RENDERER
            version = _gl_string(0x1F02)  # GL_VERSION
        except Exception as error:
            raise ProbeFailure(
                "opengl_query_failed",
                f"{type(error).__name__}: {error}",
                details={"render": info},
            ) from error
    except BaseException:
        renderer.close()
        raise
    info.update({"vendor": vendor, "renderer": renderer_name, "version": version})
    return renderer, frame


def _validate_frame(frame, info) -> ProbeFailure | None:
    import numpy as np

    if frame.dtype != np.uint8 or frame.shape != (PREVIEW_HEIGHT, PREVIEW_WIDTH, 3):
        return ProbeFailure(
            "render_frame_invalid",
            f"unexpected RGB frame dtype/shape: {frame.dtype} {frame.shape}",
            details={"render": info},
        )
    vendor = info.get("vendor")
    renderer_name = info.get("renderer")
    lowered = f"{vendor or ''} {renderer_name or ''}".lower()
    software = any(token in lowered for token in ("llvmpipe", "softpipe", "swrast"))
    if software or "nvidia" not in (vendor or "").lower():
        return ProbeFailure(
            "opengl_not_nvidia",
            f"OpenGL is not the local NVIDIA driver: vendor={vendor!r}, "
            f"renderer={renderer_name!r}",
            details={"render": info},
        )
    background = frame[:8, :8].reshape(-1, 3).astype(np.int16).mean(axis=0)
    distance = np.linalg.norm(frame.astype(np.int16) - background, axis=2)
    foreground = float((distance > 30.0).mean())
    unique = int(len(np.unique(frame.reshape(-1, 3), axis=0)))
    deviation = float(frame.std())
    info.update(
        {
            "dtype": str(frame.dtype),
            "shape": list(frame.shape),
            "mean": [round(float(value), 4) for value in frame.mean(axis=(0, 1))],
            "std": round(deviation, 4),
            "unique_colors": unique,
            "foreground_fraction": round(foreground, 6),
            "pixel_sha256": hashlib.sha256(frame.tobytes()).hexdigest(),
        }
    )
    if deviation < 1.0 or unique < 32 or foreground < 0.02:
        return ProbeFailure(
            "render_not_visible",
            "rendered frame is uniform or contains no visible model",
            details={"render": info},
        )
    return None


def _save_preview(run_dir: Path, frame, info) -> None:
    from PIL import Image

    artifacts = run_dir / "artifacts"
    artifacts.mkdir(exist_ok=False)
    png_path = artifacts / "panda_preview.png"
    Image.fromarray(frame, mode="RGB").save(png_path, format="PNG")
    body = png_path.read_bytes()
    info.update(
        {
            "artifact": png_path.relative_to(run_dir).as_posix(),
            "png_sha256": hashlib.sha256(body).hexdigest(),
            "png_bytes": len(body),
        }
    )


def _close_renderer(renderer, failure, info):
    if renderer is None:
        return failure
    try:
        renderer.close()
    except Exception as error:
        close_error = f"{type(error).__name__}: {error}"
        info["close_error"] = close_error
        if failure is None:
            return ProbeFailure(
                "renderer_close_failed",
                f"renderer close failed: {close_error}",
                details={"render": info},
            )
        failure.details.setdefault("render", info)["close_error"] = close_error
    return failure


def _render_preview(run_dir: Path) -> dict[str, object]:
    info: dict[str, object] = {}
    renderer = None
    failure: ProbeFailure | None = None
    try:
        model, data, camera, info = _prepare_render()
        renderer, frame = _capture_render(model, data, camera, info)
        failure = _validate_frame(frame, info)
        if failure is None:
            _save_preview(run_dir, frame, info)
    except ProbeFailure as error:
        failure = error
    except Exception as error:
        failure = ProbeFailure(
            "render_failed",
            f"{type(error).__name__}: {error}",
            details={"render": info},
        )
    finally:
        failure = _close_renderer(renderer, failure, info)
    if failure is not None:
        failure.details.setdefault("render", info)
        raise failure
    return info


def _emit(
    logger: object, event: str, message: str, fields: dict[str, object] | None = None
) -> None:
    try:
        logger.emit(event, message=message, fields=fields, critical=True)
    except Exception as error:
        raise ProbeFailure(
            "event_log_failed",
            f"{event}: {type(error).__name__}: {error}",
        ) from error


def _write_json_atomic(path: Path, document: object) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    body = (json.dumps(document, indent=2, sort_keys=True, allow_nan=False) + "\n").encode()
    handle = tempfile.NamedTemporaryFile(
        mode="wb", dir=path.parent, prefix=f".{path.name}.", delete=False
    )
    temporary = Path(handle.name)
    try:
        with handle:
            handle.write(body)
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(temporary, path)
    except BaseException:
        try:
            temporary.unlink()
        except OSError:
            pass
        raise


def _report_template(arguments: ProbeArguments) -> dict[str, object]:
    return {
        "schema_version": REPORT_SCHEMA_VERSION,
        "run_id": arguments.run_id,
        "component": COMPONENT,
        "instance_id": None,
        "status": "failed",
        "exit_code": 1,
        "reason": "not_started",
        "started_at_utc": _utc_now(),
        "python": {
            "implementation": platform.python_implementation(),
            "version": platform.python_version(),
            "executable": sys.executable,
        },
        "packages": None,
        "imports": None,
        "environment": {
            "MUJOCO_GL": os.environ.get("MUJOCO_GL"),
            "PYOPENGL_PLATFORM": os.environ.get("PYOPENGL_PLATFORM"),
        },
        "assets": None,
        "source": {"model_directory": str(assets.robot_model_path("panda").parent)},
        "model": None,
        "qp": None,
        "render": None,
        "artifact": None,
        "log_directory": None,
        "logs_closed": False,
        "error": None,
    }


def _open_probe_logger(arguments: ProbeArguments, run_dir: Path):
    from robot_run_logging import EventLogger, LogSettings

    instance_id = str(uuid.uuid4())
    return EventLogger(
        run_dir / "logs" / COMPONENT / instance_id,
        run_id=arguments.run_id,
        component=COMPONENT,
        instance_id=instance_id,
        settings=LogSettings(
            queue_capacity=256,
            max_event_bytes=32768,
            max_file_bytes=8 * 1024 * 1024,
            max_total_bytes=32 * 1024 * 1024,
            close_timeout_s=5.0,
            flush_interval_s=0.2,
            console=False,
        ),
    )


def _run_probe_checks(arguments, run_dir: Path, report: dict, logger) -> None:
    _emit(logger, "probe_start", "environment probe started", {"run_id": arguments.run_id})
    dependencies = check_dependencies()
    report["python"] = dependencies["python"]
    report["packages"] = dependencies["packages"]
    report["imports"] = dependencies["imports"]
    _verify_mounted_source(dependencies)
    _emit(
        logger,
        "dependencies_checked",
        "installed Python packages and import sources inspected",
        {
            "python": str(dependencies["python"]["version"]),
            "count": len(dependencies["packages"]),
        },
    )
    model_info = _check_model()
    report["model"] = model_info
    report["assets"] = {"model_directory": str(assets.robot_model_path("panda").parent)}
    stats = model_info["stats"]
    _emit(
        logger,
        "model_checked",
        "upstream Panda scene loaded from its keyframe and stepped",
        {
            "nq": int(stats["nq"]),
            "nv": int(stats["nv"]),
            "nu": int(stats["nu"]),
            "steps": MODEL_STEPS,
        },
    )
    qp_info = _check_qp()
    report["qp"] = qp_info
    _emit(
        logger,
        "qp_checked",
        "known positive-definite QP solved with DAQP",
        {"solver": "daqp", "max_abs_error": float(qp_info["max_abs_error"])},
    )
    render_info = _render_preview(run_dir)
    report["render"] = render_info
    report["artifact"] = str(render_info["artifact"])
    _emit(
        logger,
        "render_checked",
        "EGL preview rendered with the local NVIDIA driver",
        {
            "width": PREVIEW_WIDTH,
            "height": PREVIEW_HEIGHT,
            "vendor": str(render_info["vendor"]),
            "renderer": str(render_info["renderer"]),
            "unique_colors": int(render_info["unique_colors"]),
        },
    )
    try:
        logger.check_health()
    except Exception as error:
        raise ProbeFailure(
            "event_log_failed",
            f"logger health check: {type(error).__name__}: {error}",
        ) from error
    _emit(logger, "probe_complete", "environment probe checks completed")


def _merge_failure_details(error: ProbeFailure, report: dict) -> None:
    if isinstance(error.details.get("render"), dict):
        report["render"] = error.details["render"]
    if isinstance(error.details.get("packages"), dict) and report.get("packages") is None:
        report["packages"] = error.details["packages"]
    if isinstance(error.details.get("dependencies"), dict):
        dependencies = error.details["dependencies"]
        report["python"] = dependencies.get("python")
        report["packages"] = dependencies.get("packages")
        report["imports"] = dependencies.get("imports")


def _close_probe_logger(state: ProbeState) -> None:
    logger = state.logger
    if logger is None:
        return
    if state.failure is not None:
        try:
            logger.emit(
                "probe_failed",
                message="environment probe failed",
                reason=state.failure.reason,
                fields={"reason": state.failure.reason},
                critical=True,
            )
        except Exception:
            pass
    try:
        logger.close()
    except Exception as error:
        state.log_close_error = f"{type(error).__name__}: {error}"


def _finalize_report(state: ProbeState, report: dict, run_dir: Path, started: float) -> int:
    if state.log_close_error is not None and state.failure is None:
        state.failure = ProbeFailure(
            "logger_close_failed",
            f"event logger close failed: {state.log_close_error}",
        )
    report["logs_closed"] = state.logger is not None and state.log_close_error is None
    report["elapsed_s"] = round(time.monotonic() - started, 3)
    report["finished_at_utc"] = _utc_now()
    if state.failure is None:
        report["status"] = "succeeded"
        report["exit_code"] = 0
        report["reason"] = None
    else:
        report["status"] = "failed"
        report["exit_code"] = 1
        report["reason"] = state.failure.reason
        report["error"] = {
            "type": type(state.failure).__name__,
            "message": state.failure.message[:2000],
        }
    report_path = run_dir / "report.json"
    try:
        _write_json_atomic(report_path, report)
    except Exception as error:
        print(f"error: report_write_failed: {type(error).__name__}: {error}", file=sys.stderr)
        return 1
    print(f"report: {report_path}")
    return int(report["exit_code"])


def run(arguments: ProbeArguments) -> int:
    run_dir = arguments.report_root / arguments.run_id
    if os.path.lexists(run_dir):
        raise ProbeUsageError(f"report run directory already exists: {run_dir}")
    arguments.report_root.mkdir(parents=True, exist_ok=True)
    run_dir.mkdir(exist_ok=False)
    started = time.monotonic()
    report = _report_template(arguments)
    state = ProbeState(report=report)
    try:
        logger = _open_probe_logger(arguments, run_dir)
        state.logger = logger
        report["instance_id"] = logger.instance_id
        report["log_directory"] = Path(logger.directory).relative_to(run_dir).as_posix()
        _run_probe_checks(arguments, run_dir, report, logger)
    except ProbeFailure as error:
        state.failure = error
        _merge_failure_details(error, report)
    except BaseException as error:
        state.failure = ProbeFailure(
            "unexpected_error",
            f"{type(error).__name__}: {error}",
        )
    finally:
        _close_probe_logger(state)
    return _finalize_report(state, report, run_dir, started)


def main(argv: list[str] | None = None) -> int:
    try:
        arguments = parse_arguments(argv)
    except ProbeUsageError as error:
        print(f"error: {error}", file=sys.stderr)
        return 2
    try:
        return run(arguments)
    except ProbeUsageError as error:
        print(f"error: {error}", file=sys.stderr)
        return 2
    except OSError as error:
        print(f"error: report_io_error: {error}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
