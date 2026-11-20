"""Generate an installable, framework-independent SDK from the canonical proto."""

import argparse
import hashlib
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path


def generate(root: Path, output: Path) -> None:
    source = root / "shared/common/interfaces/policy/policy.proto"
    output.mkdir(parents=True, exist_ok=True)
    # A previous SDK must never be mistaken for this source revision.
    stale = output / "dual_arm_policy"
    if stale.exists():
        shutil.rmtree(stale)
    # Compile from a scratch directory so no proto copy remains in the SDK.
    with tempfile.TemporaryDirectory(prefix="policy-sdk-proto-") as scratch:
        staging = Path(scratch) / "dual_arm_policy"
        staging.mkdir()
        shutil.copyfile(source, staging / "policy.proto")
        subprocess.run(
            [
                sys.executable,
                "-m",
                "grpc_tools.protoc",
                f"--proto_path={scratch}",
                f"--python_out={output}",
                f"--grpc_python_out={output}",
                str(staging / "policy.proto"),
            ],
            check=True,
        )
    (output / "dual_arm_policy/__init__.py").write_text('"""Generated Policy API SDK."""\n')
    (output / "source.sha256").write_text(hashlib.sha256(source.read_bytes()).hexdigest() + "\n")
    print(f"Generated Policy API SDK: {output}")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=Path("build/generated/policy-sdk"))
    args = parser.parse_args()
    generate(Path(__file__).resolve().parents[4], args.output.resolve())


if __name__ == "__main__":
    main()
