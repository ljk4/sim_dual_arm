"""Run and verify the CPU-only two-container demo without a Compose dependency."""

import argparse
import json
import re
import subprocess
import uuid
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]


def docker(
    *arguments: str, capture: bool = False, timeout: float = 120
) -> subprocess.CompletedProcess:
    return subprocess.run(
        ["docker", *arguments],
        check=True,
        text=True,
        stdout=subprocess.PIPE if capture else None,
        timeout=timeout,
    )


def run(project: str, output: Path, probe_backend: bool = False) -> None:
    network, volume = project, f"{project}-episodes"
    backend, runtime = f"{project}-backend", f"{project}-runtime"
    created_containers = []
    network_created = volume_created = exported = False
    try:
        docker("network", "create", network, capture=True)
        network_created = True
        exists = subprocess.run(
            ["docker", "volume", "inspect", volume],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            check=False,
        )
        if exists.returncode == 0:
            raise RuntimeError(f"Refusing to reuse an existing test volume: {volume}")
        docker("volume", "create", volume, capture=True)
        volume_created = True
        docker(
            "create",
            "--name",
            backend,
            "--network",
            network,
            "-e",
            "ROS_DOMAIN_ID=42",
            "sim-dual-arm/backend-mock:stage2",
            capture=True,
        )
        created_containers.append(backend)
        docker("start", backend, capture=True)
        if probe_backend:
            docker(
                "run",
                "--rm",
                "--network",
                network,
                "-e",
                "ROS_DOMAIN_ID=42",
                "sim-dual-arm/runtime-test:stage2",
                "python",
                "/workspace/tests/integration/probe_backend.py",
            )
        docker(
            "create",
            "--name",
            runtime,
            "--network",
            network,
            "-e",
            "ROS_DOMAIN_ID=42",
            "--mount",
            f"type=volume,source={volume},target=/data/raw",
            "sim-dual-arm/runtime:stage2",
            capture=True,
        )
        created_containers.append(runtime)
        docker("start", runtime, capture=True)
        result = docker("wait", runtime, capture=True, timeout=60)
        docker("logs", runtime)
        output.mkdir(parents=True, exist_ok=False)
        docker("cp", f"{runtime}:/data/raw/.", str(output))
        exported = True
        if result.stdout.strip() != "0":
            docker("logs", backend)
            raise RuntimeError(
                f"Runtime exited {result.stdout.strip()}; data preserved at {output}"
            )
        episodes = sorted(path for path in output.iterdir() if path.name.startswith("episode-"))
        if len(episodes) != 1:
            raise RuntimeError(f"Expected one committed episode, found {len(episodes)}")
        report = docker(
            "run",
            "--rm",
            "--network",
            "none",
            "--mount",
            f"type=volume,source={volume},target=/data/raw,readonly",
            "sim-dual-arm/runtime:stage2",
            "python",
            "-m",
            "dual_arm_runtime.audit",
            f"/data/raw/{episodes[0].name}",
            capture=True,
        )
        print(report.stdout)
        print(json.dumps({"exported_episode": str(episodes[0])}, indent=2))
    finally:
        for container in reversed(created_containers):
            subprocess.run(
                ["docker", "rm", "-f", container],
                check=False,
                stdout=subprocess.DEVNULL,
            )
        if network_created:
            subprocess.run(
                ["docker", "network", "rm", network],
                check=False,
                stdout=subprocess.DEVNULL,
            )
        # Only remove our new volume after copying its contents to the project directory.
        if volume_created and exported:
            subprocess.run(
                ["docker", "volume", "rm", volume],
                check=False,
                stdout=subprocess.DEVNULL,
            )
        elif volume_created:
            print(f"Unexported data retained in Docker volume: {volume}")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--project", default=f"sim-dual-arm-check-{uuid.uuid4().hex[:8]}")
    parser.add_argument("--output", type=Path)
    parser.add_argument(
        "--probe-backend",
        action="store_true",
        help="Also run public ROS control checks using the test image",
    )
    args = parser.parse_args()
    if not re.fullmatch(r"[a-z0-9][a-z0-9-]{1,60}", args.project):
        parser.error("Project name must contain 2-61 lowercase letters, digits or hyphens")
    output = args.output or ROOT / "var/runs" / args.project
    if output.exists():
        parser.error(f"Output already exists: {output}")
    run(args.project, output.resolve(), args.probe_backend)


if __name__ == "__main__":
    main()
