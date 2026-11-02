#!/usr/bin/env python3
"""Run one reserved applied-label replay through the shared task launcher.

The source is test-only and uses no object truth. Campaign owns the attempt
budget; stack_once checks seed/profile, mounts the source read-only, and owns
Compose, actual feedback, independent judgement and export.
"""

from __future__ import annotations

import argparse
import os
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "tools/run"))
import stack_once  # noqa: E402


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--labels", type=Path, required=True)
    args, forwarded = parser.parse_known_args(argv)
    if os.environ.get("STACK_CAMPAIGN_KIND") != "label_replay":
        parser.error("reserve --kind label_replay in stack_campaign.py first")
    return stack_once.main(
        [
            "--replay-labels",
            str(args.labels),
            "--policy-config",
            str(ROOT / "configs/learning/replay_policy.yaml"),
            "--runtime-config",
            str(ROOT / "configs/runtime/native_replay.yaml"),
            *forwarded,
        ]
    )


if __name__ == "__main__":
    raise SystemExit(main())
