"""Build or verify the explicitly mounted Policy SDK without an installed fallback."""

from __future__ import annotations

import argparse
import fcntl
import hashlib
import importlib
import json
import os
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[4]
GENERATED_NAMES = {"dual_arm_policy", "source.sha256"}
REQUIRED_FILES = (
    "dual_arm_policy/__init__.py",
    "dual_arm_policy/policy_pb2.py",
    "dual_arm_policy/policy_pb2_grpc.py",
)
MODULES = ("dual_arm_policy.policy_pb2", "dual_arm_policy.policy_pb2_grpc")


class SDKError(RuntimeError):
    """A missing, stale or incorrectly loaded generated SDK."""


def proto_path(project_root: Path) -> Path:
    candidate = project_root / "shared/common/interfaces/policy/policy.proto"
    if candidate.is_file():
        return candidate
    shared_root = os.environ.get("SHARED_ROOT")
    if shared_root:
        shared_candidate = Path(shared_root) / "common/interfaces/policy/policy.proto"
        if shared_candidate.is_file():
            return shared_candidate
    return candidate


def verify_sdk(project_root: Path, sdk_root: Path) -> dict:
    proto = proto_path(project_root)
    root = sdk_root.resolve()
    if not proto.is_file():
        raise SDKError(f"SOURCE_MOUNT_REQUIRED: missing {proto}; mount the repository read-only")
    marker = root / "source.sha256"
    if not marker.is_file():
        raise SDKError(
            f"POLICY_SDK_MISSING: {marker}; run scripts/build_sdk.sh "
            "inside the runtime container"
        )
    expected = hashlib.sha256(proto.read_bytes()).hexdigest()
    if marker.read_text().strip() != expected:
        raise SDKError("POLICY_SDK_STALE: proto changed; regenerate SDK and restart the service")
    for relative in REQUIRED_FILES:
        path = root / relative
        if not path.is_file() or not path.resolve().is_relative_to(root):
            raise SDKError(f"POLICY_SDK_INCOMPLETE: {path}")
    sys.path.insert(0, str(root))
    origins = {}
    try:
        for name in MODULES:
            module = importlib.import_module(name)
            origin = Path(module.__file__).resolve()
            if not origin.is_relative_to(root):
                raise SDKError(f"POLICY_SDK_FALLBACK: {name} loaded from {origin}, expected {root}")
            origins[name] = str(origin)
    except (ImportError, AttributeError, TypeError, SyntaxError) as error:
        raise SDKError(f"POLICY_SDK_IMPORT_FAILED: {error}") from error
    return {"status": "verified", "source_sha256": expected, "module_origins": origins}


def prepare_output(project_root: Path, sdk_root: Path) -> Path:
    root = sdk_root.resolve()
    project = project_root.resolve()
    if root in {Path("/"), Path.home(), project} or root in project.parents:
        raise SDKError(f"UNSAFE_OUTPUT: {root}")
    if root.exists() and not root.is_dir():
        raise SDKError(f"OUTPUT_NOT_DIRECTORY: {root}")
    root.mkdir(parents=True, exist_ok=True)
    if any(path.is_symlink() for path in root.iterdir()):
        raise SDKError(f"UNOWNED_OUTPUT: symlink in {root}")
    unknown = {path.name for path in root.iterdir()} - GENERATED_NAMES
    if unknown:
        raise SDKError(f"UNOWNED_OUTPUT: unknown files in {root}: {sorted(unknown)}")
    return root


def check_command(project_root: Path, sdk_root: Path) -> list[str]:
    return [
        sys.executable,
        str(Path(__file__).resolve()),
        "check",
        "--project-root",
        str(project_root),
        "--sdk-root",
        str(sdk_root),
    ]


def build_sdk(project_root: Path, sdk_root: Path) -> None:
    root = prepare_output(project_root, sdk_root)
    # Lock the directory inode, leaving only generator-owned files inside it.
    descriptor = os.open(root, os.O_RDONLY | os.O_DIRECTORY)
    try:
        fcntl.flock(descriptor, fcntl.LOCK_EX | fcntl.LOCK_NB)
        # Recheck after taking the lock; another builder may have finished.
        prepare_output(project_root, root)
        checked = subprocess.run(check_command(project_root, root), capture_output=True, text=True)
        if checked.returncode == 0:
            print(checked.stdout, end="")
            return
        subprocess.run(
            [
                sys.executable,
                str(Path(__file__).resolve().with_name("generate_policy_sdk.py")),
                "--output",
                str(root),
            ],
            check=True,
        )
        subprocess.run(check_command(project_root, root), check=True)
    except BlockingIOError as error:
        raise SDKError(f"BUILD_BUSY: another builder owns {root}") from error
    finally:
        os.close(descriptor)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("command", choices=("check", "build"))
    parser.add_argument("--project-root", type=Path, default=ROOT)
    parser.add_argument(
        "--sdk-root", type=Path, help="default: <project-root>/build/generated/policy-sdk"
    )
    arguments = parser.parse_args()
    sdk_root = arguments.sdk_root or arguments.project_root / "build/generated/policy-sdk"
    try:
        if arguments.command == "check":
            print(
                json.dumps(verify_sdk(arguments.project_root, sdk_root), sort_keys=True)
            )
        else:
            build_sdk(arguments.project_root, sdk_root)
    except (SDKError, OSError, ValueError, subprocess.CalledProcessError) as error:
        print(f"Policy SDK: {error}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
