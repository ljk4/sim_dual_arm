"""Pinned upstream asset description and content verification.

This module deliberately depends only on the standard library.  The host fetch
tool uses it to create a stable content summary, while the container probe uses
the same rules to verify a read-only mounted checkout without relying on the
host's absolute paths.
"""

from __future__ import annotations

import hashlib
import json
import os
import re
import tempfile
from pathlib import Path, PurePosixPath

SOURCE_SCHEMA_VERSION = 1
SUMMARY_SCHEMA_VERSION = 1
CHUNK_SIZE = 1024 * 1024
COMMIT_PATTERN = re.compile(r"^[0-9a-f]{40}$")
NAME_PATTERN = re.compile(r"^[A-Za-z0-9][A-Za-z0-9._-]{0,127}$")
_POSIX_PATH = re.compile(r"^[A-Za-z0-9][A-Za-z0-9._/-]*$")


class AssetError(RuntimeError):
    """A stable asset description, checkout or summary problem."""


def _require_text(value: object, label: str, *, pattern: re.Pattern[str] | None = None) -> str:
    if not isinstance(value, str) or not value:
        raise AssetError(f"{label} must be a non-empty string")
    if pattern is not None and not pattern.fullmatch(value):
        raise AssetError(f"{label} contains unsupported characters")
    return value


def validate_commit(value: object) -> str:
    if not isinstance(value, str) or not COMMIT_PATTERN.fullmatch(value):
        raise AssetError("commit must be a full 40-character lowercase hexadecimal SHA")
    return value


def validate_relative_path(value: object, label: str) -> str:
    """Validate a repository-relative POSIX path with no escape or symlink hint."""
    text = _require_text(value, label)
    if "\\" in text or text.startswith("/"):
        raise AssetError(f"{label} must be a relative POSIX path")
    if not _POSIX_PATH.fullmatch(text):
        raise AssetError(f"{label} contains unsupported characters")
    parts = PurePosixPath(text).parts
    if not parts or any(part in {"", ".", ".."} for part in parts):
        raise AssetError(f"{label} must not contain '.', '..' or empty components")
    if ".git" in parts:
        raise AssetError(f"{label} must not point into Git metadata")
    return PurePosixPath(*parts).as_posix()


def validate_subdirectory(value: object) -> str:
    return validate_relative_path(value, "subdirectory")


def validate_repository(value: object) -> str:
    text = _require_text(value, "repository")
    if text.startswith("https://") or text.startswith("ssh://") or text.startswith("file://"):
        return text
    raise AssetError("repository must use https://, ssh:// or file://")


def load_source(path: Path) -> dict[str, object]:
    """Read and validate the single fixed-source description."""
    try:
        document = json.loads(Path(path).read_text(encoding="utf-8"))
    except FileNotFoundError as error:
        raise AssetError(f"source description is missing: {path}") from error
    except (OSError, UnicodeError, json.JSONDecodeError) as error:
        raise AssetError(f"cannot read source description {path}: {error}") from error
    if not isinstance(document, dict):
        raise AssetError("source description must be a JSON object")
    if isinstance(document.get("schema_version"), bool) or (
        document.get("schema_version") != SOURCE_SCHEMA_VERSION
    ):
        raise AssetError(f"source schema_version must be {SOURCE_SCHEMA_VERSION}")
    name = _require_text(document.get("name"), "name", pattern=NAME_PATTERN)
    repository = validate_repository(document.get("repository"))
    commit = validate_commit(document.get("commit"))
    subdirectory = validate_subdirectory(document.get("subdirectory"))
    license_name = _require_text(document.get("license"), "license")
    license_path = validate_relative_path(document.get("license_path"), "license_path")
    return {
        "schema_version": SOURCE_SCHEMA_VERSION,
        "name": name,
        "repository": repository,
        "commit": commit,
        "subdirectory": subdirectory,
        "license": license_name,
        "license_path": license_path,
    }


def _sha256_file(path: Path) -> tuple[int, str]:
    digest = hashlib.sha256()
    size = 0
    with path.open("rb") as stream:
        while True:
            chunk = stream.read(CHUNK_SIZE)
            if not chunk:
                break
            size += len(chunk)
            digest.update(chunk)
    return size, digest.hexdigest()


def _reject_link(path: Path, label: str) -> None:
    if path.is_symlink():
        raise AssetError(f"{label} is an unsupported symbolic link: {path}")


def _scan_files(asset_root: Path, subdirectory: str) -> list[dict[str, object]]:
    provided = Path(asset_root)
    _reject_link(provided, "assets root")
    root = provided.resolve()
    directory = root / subdirectory
    if not directory.is_dir():
        raise AssetError(f"asset subdirectory is missing: {subdirectory}")
    _reject_link(directory, "asset subdirectory")
    entries: list[dict[str, object]] = []
    for current, directory_names, file_names in os.walk(directory, followlinks=False):
        current_path = Path(current)
        for name in sorted(directory_names):
            candidate = current_path / name
            if candidate.is_symlink():
                relative = candidate.relative_to(root)
                raise AssetError(f"asset directory is a symbolic link: {relative}")
        for name in sorted(file_names):
            candidate = current_path / name
            if candidate.is_symlink():
                raise AssetError(f"asset file is a symbolic link: {candidate.relative_to(root)}")
            try:
                candidate.relative_to(root)
            except ValueError as error:
                raise AssetError(f"asset path escapes the checkout: {candidate}") from error
            if not candidate.is_file():
                raise AssetError(f"asset entry is not a regular file: {candidate}")
            relative = candidate.relative_to(directory).as_posix()
            validate_relative_path(relative, "asset file path")
            size, sha256 = _sha256_file(candidate)
            entries.append({"path": relative, "size": size, "sha256": sha256})
    if not entries:
        raise AssetError(f"asset subdirectory is empty: {subdirectory}")
    entries.sort(key=lambda entry: str(entry["path"]))
    return entries


def _canonical_json(document: object) -> bytes:
    return json.dumps(
        document, ensure_ascii=False, allow_nan=False, sort_keys=True, separators=(",", ":")
    ).encode("utf-8")


def total_digest(subdirectory: str, files: list[dict[str, object]]) -> str:
    document = {"subdirectory": subdirectory, "files": files}
    return hashlib.sha256(_canonical_json(document)).hexdigest()


def build_manifest(asset_root: Path, source: dict[str, object]) -> dict[str, object]:
    subdirectory = validate_subdirectory(source["subdirectory"])
    files = _scan_files(Path(asset_root), subdirectory)
    paths = {str(entry["path"]) for entry in files}
    license_path = str(source["license_path"])
    if license_path not in paths:
        raise AssetError(f"license file is missing from the asset subdirectory: {license_path}")
    return {
        "schema_version": SUMMARY_SCHEMA_VERSION,
        "subdirectory": subdirectory,
        "files": files,
        "total_sha256": total_digest(subdirectory, files),
    }


def summary_document(source: dict[str, object], manifest: dict[str, object]) -> dict[str, object]:
    return {
        "schema_version": SUMMARY_SCHEMA_VERSION,
        "asset": source["name"],
        "source": {
            "repository": source["repository"],
            "commit": source["commit"],
            "subdirectory": source["subdirectory"],
            "license": source["license"],
            "license_path": source["license_path"],
        },
        "files": manifest["files"],
        "total_sha256": manifest["total_sha256"],
    }


def _validate_manifest_files(files: object) -> list[dict[str, object]]:
    if not isinstance(files, list):
        raise AssetError("summary files must be a list")
    result: list[dict[str, object]] = []
    seen: set[str] = set()
    for index, entry in enumerate(files):
        if not isinstance(entry, dict):
            raise AssetError(f"summary files[{index}] must be an object")
        relative = validate_relative_path(entry.get("path"), f"summary files[{index}].path")
        if relative in seen:
            raise AssetError(f"summary contains a duplicate path: {relative}")
        seen.add(relative)
        size = entry.get("size")
        if isinstance(size, bool) or not isinstance(size, int) or size < 0:
            raise AssetError(f"summary files[{index}].size must be a non-negative integer")
        sha256 = entry.get("sha256")
        if not isinstance(sha256, str) or not re.fullmatch(r"[0-9a-f]{64}", sha256):
            raise AssetError(f"summary files[{index}].sha256 must be lowercase hexadecimal")
        result.append({"path": relative, "size": size, "sha256": sha256})
    result.sort(key=lambda item: str(item["path"]))
    if not result:
        raise AssetError("summary must contain at least one file")
    return result


def load_summary(path: Path) -> dict[str, object]:
    try:
        document = json.loads(Path(path).read_text(encoding="utf-8"))
    except FileNotFoundError as error:
        raise AssetError(f"asset summary is missing: {path}") from error
    except (OSError, UnicodeError, json.JSONDecodeError) as error:
        raise AssetError(f"cannot read asset summary {path}: {error}") from error
    if not isinstance(document, dict):
        raise AssetError("asset summary must be a JSON object")
    if isinstance(document.get("schema_version"), bool) or (
        document.get("schema_version") != SUMMARY_SCHEMA_VERSION
    ):
        raise AssetError(f"asset summary schema_version must be {SUMMARY_SCHEMA_VERSION}")
    asset = _require_text(document.get("asset"), "summary asset", pattern=NAME_PATTERN)
    source = document.get("source")
    if not isinstance(source, dict):
        raise AssetError("asset summary source must be an object")
    normalized_source = {
        "repository": validate_repository(source.get("repository")),
        "commit": validate_commit(source.get("commit")),
        "subdirectory": validate_subdirectory(source.get("subdirectory")),
        "license": _require_text(source.get("license"), "summary source license"),
        "license_path": validate_relative_path(
            source.get("license_path"), "summary source license_path"
        ),
    }
    files = _validate_manifest_files(document.get("files"))
    total = document.get("total_sha256")
    if not isinstance(total, str) or not re.fullmatch(r"[0-9a-f]{64}", total):
        raise AssetError("asset summary total_sha256 must be lowercase hexadecimal")
    if total != total_digest(normalized_source["subdirectory"], files):
        raise AssetError("asset summary total_sha256 does not match its file manifest")
    return {
        "schema_version": SUMMARY_SCHEMA_VERSION,
        "asset": asset,
        "source": normalized_source,
        "files": files,
        "total_sha256": total,
    }


def verify_assets(
    asset_root: Path,
    source: dict[str, object],
    summary_path: Path,
) -> dict[str, object]:
    """Verify a mounted checkout against the fixed source and read-only summary."""
    summary = load_summary(Path(summary_path))
    if summary["asset"] != source["name"]:
        raise AssetError(
            f"summary asset {summary['asset']!r} does not match source {source['name']!r}"
        )
    summary_source = summary["source"]
    for key in ("repository", "commit", "subdirectory", "license", "license_path"):
        if summary_source[key] != source[key]:
            raise AssetError(f"summary source {key} does not match the fixed source description")
    manifest = build_manifest(Path(asset_root), source)
    if manifest["files"] != summary["files"]:
        expected = {str(entry["path"]): entry for entry in summary["files"]}
        actual = {str(entry["path"]): entry for entry in manifest["files"]}
        missing = sorted(set(expected) - set(actual))
        extra = sorted(set(actual) - set(expected))
        changed = sorted(
            path for path in set(expected) & set(actual) if expected[path] != actual[path]
        )
        detail = []
        if missing:
            detail.append(f"missing={missing[:5]}")
        if extra:
            detail.append(f"extra={extra[:5]}")
        if changed:
            detail.append(f"changed={changed[:5]}")
        raise AssetError(
            "asset files do not match the read-only summary: " + ", ".join(detail or ["unknown"])
        )
    if manifest["total_sha256"] != summary["total_sha256"]:
        raise AssetError("asset total digest does not match the read-only summary")
    return {
        "asset": source["name"],
        "repository": source["repository"],
        "commit": source["commit"],
        "subdirectory": source["subdirectory"],
        "license": source["license"],
        "license_path": source["license_path"],
        "file_count": len(manifest["files"]),
        "total_sha256": manifest["total_sha256"],
        "summary_sha256": hashlib.sha256(Path(summary_path).read_bytes()).hexdigest(),
    }


def write_summary(path: Path, document: dict[str, object], checkout: Path) -> None:
    """Write a summary atomically, never inside the vendor checkout."""
    destination = Path(path).resolve()
    checkout_root = Path(checkout).resolve()
    if destination == checkout_root or checkout_root in destination.parents:
        raise AssetError("asset summary must not be written inside the vendor checkout")
    if destination.is_symlink():
        raise AssetError("summary path must not be a symbolic link")
    destination.parent.mkdir(parents=True, exist_ok=True)
    body = (json.dumps(document, indent=2, ensure_ascii=False, allow_nan=False) + "\n").encode()
    handle = tempfile.NamedTemporaryFile(
        mode="wb", dir=destination.parent, prefix=f".{destination.name}.", delete=False
    )
    temporary = Path(handle.name)
    try:
        with handle:
            handle.write(body)
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(temporary, destination)
    except BaseException:
        try:
            temporary.unlink()
        except OSError:
            pass
        raise


def model_paths(asset_root: Path, source: dict[str, object]) -> tuple[Path, Path]:
    directory = Path(asset_root).resolve() / validate_subdirectory(source["subdirectory"])
    return directory / "scene.xml", directory / "panda.xml"
