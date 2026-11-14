"""Shared filesystem, hashing and JSON primitives for dataset conversion."""

from __future__ import annotations

import hashlib
import json
import math
import os
from pathlib import Path
from typing import Any

import numpy as np
from robot_contracts.project_paths import project_path

from .config import PNG_SHA, DatasetConfig


class ExportError(ValueError):
    """A source export failed an auditable precondition."""


def load_json_document(path: Path) -> dict[str, Any]:
    def bad_constant(value: str) -> Any:
        raise ExportError(f"{path}: non-finite JSON constant {value!r}")

    try:
        value = json.loads(path.read_text(encoding="utf-8"), parse_constant=bad_constant)
    except OSError as error:
        raise ExportError(f"{path}: cannot read: {error}") from error
    except json.JSONDecodeError as error:
        raise ExportError(f"{path}: invalid JSON: {error}") from error
    if not isinstance(value, dict):
        raise ExportError(f"{path}: expected a JSON object")
    return value


def require_int(value: object, location: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int):
        raise ExportError(f"{location}: expected an integer")
    return value


def require_float(value: object, location: str) -> float:
    if isinstance(value, bool) or not isinstance(value, int | float):
        raise ExportError(f"{location}: expected a number")
    result = float(value)
    if not math.isfinite(result):
        raise ExportError(f"{location}: expected a finite number")
    return result


def require_vector(value: object, location: str, size: int) -> np.ndarray:
    if not isinstance(value, list) or len(value) != size:
        raise ExportError(f"{location}: expected an array of {size} numbers")
    result = np.asarray(
        [require_float(item, f"{location}[{index}]") for index, item in enumerate(value)],
        dtype=np.float32,
    )
    if not np.isfinite(result).all():
        raise ExportError(f"{location}: contains non-finite values")
    return result


def require_chunk(value: object, location: str, rows: int, columns: int) -> np.ndarray:
    if not isinstance(value, list) or len(value) != rows:
        raise ExportError(f"{location}: expected {rows} rows")
    result = np.empty((rows, columns), dtype=np.float32)
    for index, point in enumerate(value):
        result[index] = require_vector(point, f"{location}[{index}]", columns)
    if not np.isfinite(result).all():
        raise ExportError(f"{location}: contains non-finite values")
    return result


def configure_local_caches(root: Path, *, data_root: Path | None = None) -> Path:
    """Select and validate an explicit writable, offline cache root.

    ``LEARNING_CACHE_ROOT``/``HF_HOME`` override the configured path, but the
    chosen directory must be writable and must not be inside accepted data.
    No cache is ever created under a read-only dataset root.
    """
    candidate = Path(
        os.environ.get("LEARNING_CACHE_ROOT") or os.environ.get("HF_HOME") or root
    ).expanduser()
    resolved = candidate.resolve()
    if data_root is not None:
        data = Path(data_root).resolve()
        if resolved == data or resolved.is_relative_to(data):
            raise ExportError(f"cache root {resolved} must not be inside data root {data}")
    try:
        resolved.mkdir(parents=True, exist_ok=True)
        probe = resolved / f".write-probe-{os.getpid()}"
        probe.write_text("ok", encoding="utf-8")
        probe.unlink()
    except OSError as error:
        raise ExportError(f"cache root {resolved} is not writable: {error}") from error
    os.environ["HF_HOME"] = str(resolved)
    os.environ["HF_DATASETS_CACHE"] = str(resolved / "datasets")
    os.environ["HF_HUB_OFFLINE"] = "1"
    os.environ["HF_DATASETS_OFFLINE"] = "1"
    os.environ["TRANSFORMERS_OFFLINE"] = "1"
    return resolved


def file_sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def source_archive_sha256(config: DatasetConfig) -> str:
    """Return the configured archive digest or the actual source binding digest."""
    if config.lerobot_source_archive_sha256:
        return config.lerobot_source_archive_sha256
    binding = project_path("learning/lerobot/docker/learning-source.json", config.project_root)
    try:
        value = json.loads(binding.read_text(encoding="utf-8")).get("archive_sha256")
    except (OSError, json.JSONDecodeError):
        return ""
    return value if isinstance(value, str) and PNG_SHA.fullmatch(value) else ""


def canonical_hash(document: object) -> str:
    payload = json.dumps(document, sort_keys=True, separators=(",", ":"), allow_nan=False)
    return hashlib.sha256(payload.encode("utf-8")).hexdigest()


def tree_files(root: Path) -> list[Path]:
    return sorted(path for path in root.rglob("*") if path.is_file())


def tree_hashes(root: Path) -> dict[str, dict[str, Any]]:
    result: dict[str, dict[str, Any]] = {}
    for path in tree_files(root):
        result[path.relative_to(root).as_posix()] = {
            "sha256": file_sha256(path),
            "bytes": path.stat().st_size,
        }
    return result


def resolve_export_dir(attempt_dir: Path, pattern: str) -> Path:
    candidates = sorted(
        path.parent if path.name == "labels.json" else path
        for path in attempt_dir.glob(pattern)
        if (path / "labels.json").is_file() or path.name == "labels.json"
    )
    unique: list[Path] = []
    for candidate in candidates:
        if candidate not in unique:
            unique.append(candidate)
    if len(unique) != 1:
        raise ExportError(
            f"{attempt_dir}: expected exactly one accepted export matching {pattern!r}, "
            f"found {len(unique)}"
        )
    return unique[0]


__all__ = [
    "ExportError",
    "canonical_hash",
    "configure_local_caches",
    "file_sha256",
    "load_json_document",
    "require_chunk",
    "require_float",
    "require_int",
    "require_vector",
    "resolve_export_dir",
    "source_archive_sha256",
    "tree_files",
    "tree_hashes",
]
