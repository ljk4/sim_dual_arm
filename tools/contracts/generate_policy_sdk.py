"""Generate an installable, framework-independent SDK from the canonical proto."""

import argparse
import hashlib
import shutil
import subprocess
import sys
from pathlib import Path


def generate(root: Path, output: Path) -> None:
    source = root / "contracts/policy/v1/policy.proto"
    staging = output / "_proto/dual_arm_policy/v1"
    staging.mkdir(parents=True, exist_ok=True)
    shutil.copyfile(source, staging / "policy.proto")
    subprocess.run(
        [
            sys.executable,
            "-m",
            "grpc_tools.protoc",
            f"--proto_path={output / '_proto'}",
            f"--python_out={output}",
            f"--grpc_python_out={output}",
            str(staging / "policy.proto"),
        ],
        check=True,
    )
    for package in (output / "dual_arm_policy", output / "dual_arm_policy/v1"):
        (package / "__init__.py").write_text('"""Generated Policy API SDK."""\n')
    (output / "pyproject.toml").write_text(
        '[build-system]\nrequires = ["setuptools>=68", "wheel"]\n'
        'build-backend = "setuptools.build_meta"\n'
        '[project]\nname = "dual-arm-policy-sdk"\nversion = "0.1.0"\n'
        'requires-python = ">=3.12"\n'
        'dependencies = ["grpcio>=1.73.1,<2", "protobuf>=6.31.1,<7"]\n'
        '[tool.setuptools.packages.find]\ninclude = ["dual_arm_policy*"]\n'
    )
    (output / "source.sha256").write_text(hashlib.sha256(source.read_bytes()).hexdigest() + "\n")
    print(f"Generated Policy API SDK: {output}")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=Path(".build/generated/policy-sdk"))
    args = parser.parse_args()
    generate(Path(__file__).resolve().parents[2], args.output.resolve())


if __name__ == "__main__":
    main()
