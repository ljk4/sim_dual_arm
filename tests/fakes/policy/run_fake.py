"""Container/host entry point that injects the fake provider into the service."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

from dual_arm_learning.policy_server import serve_forever
from fake_provider import FakeProvider, load_fake_config


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--config",
        type=Path,
        default=Path("/workspace/learning/lerobot/configs/fake_policy.yaml"),
    )
    parser.add_argument("--log-directory", type=Path, default=Path("var/logs/policy"))
    parser.add_argument("--run-id")
    args = parser.parse_args(argv)
    config, settings = load_fake_config(args.config)
    provider = FakeProvider(config, settings)
    return serve_forever(config, provider, log_directory=args.log_directory, run_id=args.run_id)


if __name__ == "__main__":
    sys.exit(main())
