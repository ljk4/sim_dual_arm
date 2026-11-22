"""Shared deterministic JSON, YAML and hashing helpers across all workspace tiers."""

from __future__ import annotations

import hashlib
import json
import os
from pathlib import Path
from typing import Any

from .models import ContractError
from .validation import UniqueKeyError, read_unique_yaml

__all__ = [
    "canonical_sha256",
    "read_yaml_unique_mapping",
    "sha256_file",
    "write_json_atomic",
]


def sha256_file(path: Path) -> str:
    """Return the lowercase SHA-256 hex digest of a file using buffered reads."""
    digest = hashlib.sha256()
    with Path(path).open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def canonical_sha256(document: object) -> str:
    """Return the canonical compact-JSON SHA-256 hex digest of an object."""
    payload = json.dumps(document, sort_keys=True, separators=(",", ":"), allow_nan=False)
    return hashlib.sha256(payload.encode("utf-8")).hexdigest()


def write_json_atomic(path: Path, document: object, *, indent: int | None = 2) -> None:
    """Atomically write a finite JSON document via same-directory temporary replace."""
    target = Path(path)
    target.parent.mkdir(parents=True, exist_ok=True)
    temporary = target.with_name(f".{target.name}.{os.getpid()}.tmp")
    text = json.dumps(document, indent=indent, sort_keys=True, allow_nan=False) + "\n"
    try:
        temporary.write_text(text, encoding="utf-8")
        os.replace(temporary, target)
    except Exception:
        temporary.unlink(missing_ok=True)
        raise


def read_yaml_unique_mapping(
    path: Path | str,
    *,
    label: str = "document",
    error_cls: type[Exception] = ContractError,
) -> dict[str, Any]:
    """Read a YAML file rejecting duplicate mapping keys and non-object roots."""
    source = Path(path)
    try:
        document = read_unique_yaml(source)
    except (UniqueKeyError, OSError) as error:
        if error_cls is ContractError:
            raise ContractError("SCHEMA_MISMATCH", f"cannot read {label} {source}: {error}") from error
        raise error_cls(f"cannot read {label} {source}: {error}") from error
    if not isinstance(document, dict):
        if error_cls is ContractError:
            raise ContractError("SCHEMA_MISMATCH", f"{label} must be a YAML object: {source}")
        raise error_cls(f"{label} must be a YAML object: {source}")
    return document
