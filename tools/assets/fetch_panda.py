#!/usr/bin/env python3
"""Fetch and verify the pinned MuJoCo Menagerie Panda checkout.

Only the standard library and the host ``git`` executable are used.  The
checkout lives under the ignored ``third_party/src/`` tree; the content summary
is written separately under ``.build/assets/`` and never inside the checkout.
"""

from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
import uuid
from collections.abc import Sequence
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
COMPONENT_SRC = ROOT / "components/backend_sim/src"
if str(COMPONENT_SRC) not in sys.path:
    sys.path.insert(0, str(COMPONENT_SRC))

from robot_backend_sim import assets  # noqa: E402

DEFAULT_SOURCE = ROOT / "third_party/panda-source.json"
DEFAULT_CHECKOUT = ROOT / "third_party/src/mujoco_menagerie"
DEFAULT_SUMMARY = ROOT / ".build/assets/panda.json"
GIT_TIMEOUT_S = 180


class FetchError(RuntimeError):
    """A stable, user-actionable fetch/verify failure."""

    def __init__(self, reason: str, detail: str = "") -> None:
        super().__init__(detail or reason)
        self.reason = reason
        self.detail = detail


def _validate_checkout(path: Path) -> Path:
    provided = Path(path)
    if provided.is_symlink():
        raise FetchError("invalid_checkout", f"checkout must not be a symbolic link: {provided}")
    resolved = provided.resolve()
    if resolved == Path("/"):
        raise FetchError("invalid_checkout", "checkout must not be the filesystem root")
    if resolved == ROOT or resolved in ROOT.parents:
        raise FetchError("invalid_checkout", "checkout must not be the repository root or a parent")
    home = Path.home().resolve()
    if resolved == home or resolved in home.parents:
        raise FetchError("invalid_checkout", "checkout must not be the home directory or a parent")
    return resolved


def _git(
    arguments: Sequence[str],
    *,
    cwd: Path,
    check: bool = True,
    timeout: int = GIT_TIMEOUT_S,
) -> subprocess.CompletedProcess[str]:
    argv = ["git", *arguments]
    try:
        completed = subprocess.run(
            argv,
            cwd=cwd,
            check=False,
            capture_output=True,
            text=True,
            timeout=timeout,
        )
    except FileNotFoundError as error:
        raise FetchError(
            "git_unavailable", "git is required to fetch the pinned checkout"
        ) from error
    except subprocess.TimeoutExpired as error:
        raise FetchError(
            "git_timeout", f"git command exceeded {timeout}s: {arguments[0]}"
        ) from error
    except OSError as error:
        raise FetchError("git_error", f"{type(error).__name__}: {error}") from error
    if check and completed.returncode != 0:
        detail = (completed.stderr or completed.stdout or "").strip()
        if len(detail) > 1200:
            detail = detail[-1200:]
        raise FetchError(
            "git_error",
            f"git {arguments[0]} exited {completed.returncode}: {detail}",
        )
    return completed


def _head(checkout: Path) -> str:
    return _git(["rev-parse", "HEAD"], cwd=checkout).stdout.strip()


def _origin(checkout: Path) -> str:
    return _git(["remote", "get-url", "origin"], cwd=checkout).stdout.strip()


def _require_clean_checkout(checkout: Path) -> None:
    status = _git(
        ["status", "--porcelain", "--untracked-files=all", "--ignored=matching"],
        cwd=checkout,
    ).stdout
    if status.strip():
        raise FetchError(
            "checkout_dirty",
            "existing checkout has local changes or ignored files; files were preserved",
        )


def _inspect_existing(checkout: Path, source: dict[str, object]) -> None:
    if checkout.is_symlink() or not checkout.is_dir():
        raise FetchError(
            "checkout_not_directory", f"existing checkout path is not a plain directory: {checkout}"
        )
    if not (checkout / ".git").exists():
        raise FetchError(
            "checkout_not_owned",
            f"existing directory is not this tool's Git checkout: {checkout}; files were preserved",
        )
    if _git(["rev-parse", "--is-inside-work-tree"], cwd=checkout).stdout.strip() != "true":
        raise FetchError("checkout_not_owned", f"not a Git work tree: {checkout}")
    origin = _origin(checkout)
    if origin != source["repository"]:
        raise FetchError(
            "checkout_origin_mismatch",
            f"checkout origin {origin!r} does not match {source['repository']!r}; preserved",
        )
    head = _head(checkout)
    if head != source["commit"]:
        raise FetchError(
            "checkout_commit_mismatch",
            f"checkout HEAD {head} does not match fixed commit {source['commit']}; preserved",
        )
    _require_clean_checkout(checkout)


def _clone_sparse(checkout: Path, source: dict[str, object]) -> Path:
    parent = checkout.parent
    parent.mkdir(parents=True, exist_ok=True)
    staging = parent / f".{checkout.name}.fetch-{uuid.uuid4().hex[:10]}"
    if os.path.lexists(staging):
        raise FetchError("staging_conflict", f"staging path already exists: {staging}")
    try:
        staging.mkdir()
        _git(["init", "--quiet"], cwd=staging)
        _git(["remote", "add", "origin", str(source["repository"])], cwd=staging)
        _git(["sparse-checkout", "init", "--cone"], cwd=staging)
        _git(["sparse-checkout", "set", str(source["subdirectory"])], cwd=staging)
        # Fetch the explicit commit rather than following a branch.  Partial
        # clones keep unrelated model blobs out of the local checkout.
        _git(
            ["fetch", "--depth=1", "--filter=blob:none", "origin", str(source["commit"])],
            cwd=staging,
        )
        _git(["checkout", "--detach", "FETCH_HEAD"], cwd=staging)
        head = _head(staging)
        if head != source["commit"]:
            raise FetchError(
                "fetched_commit_mismatch",
                f"fetched HEAD {head} does not match fixed commit {source['commit']}",
            )
        _require_clean_checkout(staging)
        if os.path.lexists(checkout):
            raise FetchError(
                "checkout_appeared",
                f"checkout path appeared while fetching: {checkout}; staging kept at {staging}",
            )
        os.replace(staging, checkout)
    except FetchError as error:
        # Keep the staging directory and its diagnostic Git state for the first
        # failure.  Only this invocation's staging path is ever involved.
        raise FetchError(
            error.reason,
            f"{error.detail or error.reason} (staging preserved at {staging})",
        ) from error
    except OSError as error:
        raise FetchError(
            "checkout_io_error",
            f"{type(error).__name__}: {error} (staging preserved at {staging})",
        ) from error
    return checkout


def fetch(source_path: Path, checkout: Path, summary_path: Path) -> dict[str, object]:
    source = assets.load_source(source_path)
    checkout = _validate_checkout(checkout)
    if checkout.exists():
        _inspect_existing(checkout, source)
    else:
        _clone_sparse(checkout, source)
    manifest = assets.build_manifest(checkout, source)
    document = assets.summary_document(source, manifest)
    assets.write_summary(summary_path, document, checkout)
    return {
        "action": "fetch",
        "checkout": str(checkout),
        "summary": str(Path(summary_path).resolve()),
        **{key: document[key] for key in ("asset", "source", "total_sha256")},
        "file_count": len(manifest["files"]),
    }


def verify(source_path: Path, checkout: Path, summary_path: Path) -> dict[str, object]:
    source = assets.load_source(source_path)
    checkout = _validate_checkout(checkout)
    if not checkout.is_dir():
        raise FetchError("checkout_missing", f"checkout is missing: {checkout}")
    if (checkout / ".git").exists():
        head = _head(checkout)
        if head != source["commit"]:
            raise FetchError(
                "checkout_commit_mismatch",
                f"checkout HEAD {head} does not match fixed commit {source['commit']}",
            )
    result = assets.verify_assets(checkout, source, summary_path)
    return {
        "action": "verify",
        "checkout": str(checkout),
        "summary": str(Path(summary_path).resolve()),
        **result,
    }


def _absolute(value: Path | None, default: Path) -> Path:
    path = default if value is None else value
    if not path.is_absolute():
        path = ROOT / path
    return path.resolve()


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("command", choices=("fetch", "verify"))
    parser.add_argument("--source", type=Path, help="fixed upstream source description JSON")
    parser.add_argument("--checkout", type=Path, help="Menagerie checkout root")
    parser.add_argument("--summary", type=Path, help="generated asset summary JSON")
    return parser


def main(argv: list[str] | None = None) -> int:
    arguments = _parser().parse_args(argv)
    source = _absolute(arguments.source, DEFAULT_SOURCE)
    checkout = _absolute(arguments.checkout, DEFAULT_CHECKOUT)
    summary = _absolute(arguments.summary, DEFAULT_SUMMARY)
    try:
        if arguments.command == "fetch":
            result = fetch(source, checkout, summary)
        else:
            result = verify(source, checkout, summary)
    except (FetchError, assets.AssetError) as error:
        reason = getattr(error, "reason", "asset_invalid")
        detail = getattr(error, "detail", "") or str(error)
        print(f"error: {reason}: {detail}", file=sys.stderr)
        return 1
    except OSError as error:
        print(f"error: asset_io_error: {type(error).__name__}: {error}", file=sys.stderr)
        return 1
    print(json.dumps(result, ensure_ascii=False, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
