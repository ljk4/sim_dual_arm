"""Record dependency-image provenance and verify the installed Ruckig wheel."""

from __future__ import annotations

import argparse
import hashlib
import importlib
import importlib.metadata
import json
import platform
import subprocess
import sys
from pathlib import Path


def file_sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def python_version(*, resolve_executable: bool = False) -> dict[str, str]:
    return {
        "implementation": platform.python_implementation(),
        "version": platform.python_version(),
        "executable": str(Path(sys.executable).resolve()) if resolve_executable else sys.executable,
    }


def write_json(path: Path, document: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(document, indent=2, sort_keys=True) + "\n")


def record_sim(packages: Path, output: Path) -> dict:
    specification = json.loads(packages.read_text())
    installed = {}
    for entry in specification["packages"]:
        try:
            installed[entry["name"]] = importlib.metadata.version(entry["name"])
        except importlib.metadata.PackageNotFoundError:
            installed[entry["name"]] = None
    graphics = subprocess.check_output(
        [
            "dpkg-query",
            "-W",
            "-f=${Package}=${Version}\n",
            "libegl1",
            "libgl1",
            "libopengl0",
            "libglvnd0",
        ],
        text=True,
    ).splitlines()
    document = {
        "schema_version": 1,
        "python": python_version(),
        "packages": installed,
        "graphics": dict(line.split("=", 1) for line in graphics),
    }
    write_json(output, document)
    return document


def record_ruckig(source: Path, wheel_dir: Path, lock_dir: Path, output: Path) -> dict:
    specification = json.loads(source.read_text())
    wheels = sorted(wheel_dir.glob("*.whl"))
    if len(wheels) != 1:
        raise ValueError(f"expected one Ruckig wheel in {wheel_dir}, found {len(wheels)}")
    wheel = wheels[0]
    tools = ("scikit-build-core", "nanobind", "cmake", "ninja", "packaging", "pathspec")
    locks = ("ruckig-requirements.lock", "ruckig-build-requirements.lock")
    document = {
        "schema_version": 1,
        "package": specification["package"],
        "version": specification["version"],
        "license": specification["license"],
        "source_url": specification["source_url"],
        "sdist_sha256": specification["sdist_sha256"],
        "wheel_file": wheel.name,
        "wheel_sha256": file_sha256(wheel),
        "wheel_bytes": wheel.stat().st_size,
        "python": python_version(resolve_executable=True),
        "build_tools": {name: importlib.metadata.version(name) for name in tools},
        "requirements_sha256": {name: file_sha256(lock_dir / name) for name in locks},
    }
    write_json(output, document)
    return document


def verify_ruckig(provenance: Path) -> None:
    document = json.loads(provenance.read_text())
    importlib.import_module("ruckig")
    version = importlib.metadata.version("ruckig")
    if version != document["version"]:
        raise ValueError(
            f"Ruckig version differs: installed={version}, recorded={document['version']}"
        )
    print(f"follower-dev ruckig {version} {document['wheel_sha256']}")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="command", required=True)
    sim = commands.add_parser("sim")
    sim.add_argument("--packages", type=Path, required=True)
    sim.add_argument("--output", type=Path, required=True)
    ruckig = commands.add_parser("ruckig-build")
    ruckig.add_argument("--source", type=Path, required=True)
    ruckig.add_argument("--wheel-dir", type=Path, required=True)
    ruckig.add_argument("--lock-dir", type=Path, required=True)
    ruckig.add_argument("--output", type=Path, required=True)
    check = commands.add_parser("ruckig-check")
    check.add_argument("--provenance", type=Path, required=True)
    arguments = parser.parse_args()
    try:
        if arguments.command == "sim":
            record_sim(arguments.packages, arguments.output)
        elif arguments.command == "ruckig-build":
            record_ruckig(
                arguments.source, arguments.wheel_dir, arguments.lock_dir, arguments.output
            )
        else:
            verify_ruckig(arguments.provenance)
    except (OSError, ValueError, KeyError, ImportError, subprocess.CalledProcessError) as error:
        print(f"image metadata failed: {error}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
