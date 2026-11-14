#!/usr/bin/env python3
"""Read-only scenario binding validator and viewer.

Examples:
  python3 shared/common/tools/scenario.py validate \
      --binding shared/scenarios/<scenario-id>/binding.yaml --stage collection
  python3 shared/common/tools/scenario.py show \
      --binding shared/scenarios/<scenario-id>/binding.yaml

The tool never imports ROS, MuJoCo, PyTorch or a scenario implementation, never
starts a process and never writes campaign ledgers, seeds or other state.
"""

from __future__ import annotations

import argparse
import json
import os
import sys
from pathlib import Path

_LAYOUT_ROOT = Path(__file__).resolve().parents[2]
_CONTRACTS_SRC = _LAYOUT_ROOT / "common" / "python" / "contracts" / "src"
if str(_CONTRACTS_SRC) not in sys.path:
    sys.path.insert(0, str(_CONTRACTS_SRC))

from robot_contracts.binding import (  # noqa: E402
    BindingError,
    BindingStage,
    ResolvedBinding,
    load_binding,
)
from robot_contracts.models import ContractError  # noqa: E402


def _shared_root(value: str | None) -> Path:
    if value is not None:
        return Path(value)
    environment = os.environ.get("SHARED_ROOT")
    if environment:
        return Path(environment)
    return _LAYOUT_ROOT


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Validate or display one scenario binding without executing it."
    )
    common = argparse.ArgumentParser(add_help=False)
    common.add_argument(
        "--shared-root",
        help="explicit shared tree; defaults to SHARED_ROOT or this tool's shared/ layout",
    )
    commands = parser.add_subparsers(dest="command", required=True)

    validate = commands.add_parser(
        "validate", parents=[common], help="check structure, references and a stage"
    )
    validate.add_argument("--binding", required=True, help="path to binding.yaml")
    validate.add_argument(
        "--stage",
        required=True,
        choices=[stage.value for stage in BindingStage],
        help="stage whose requirements must be satisfied",
    )
    validate.add_argument(
        "--bundle",
        help="explicit model bundle directory overriding models.bundle",
    )

    show = commands.add_parser("show", parents=[common], help="print the effective binding as JSON")
    show.add_argument("--binding", required=True, help="path to binding.yaml")
    return parser


def _emit(document: dict) -> None:
    json.dump(document, sys.stdout, indent=2, ensure_ascii=False, allow_nan=False)
    sys.stdout.write("\n")


def _failed(error: Exception) -> int:
    print(f"error: {error}", file=sys.stderr)
    return 2


def main(argv: list[str] | None = None) -> int:
    arguments = _parser().parse_args(argv)
    try:
        resolved: ResolvedBinding = load_binding(
            arguments.binding,
            shared_root=_shared_root(arguments.shared_root),
            bundle=getattr(arguments, "bundle", None),
        )
        if arguments.command == "validate":
            stage = BindingStage(arguments.stage)
            resolved.validate_stage(stage)
            _emit(
                {
                    "ok": True,
                    "stage": stage.value,
                    "binding": str(resolved.path),
                    "binding_sha256": resolved.sha256,
                    "scenario_id": resolved.binding.scenario_id,
                    "models": resolved.effective_bundle(),
                    "checked": list(resolved.requirements(stage)),
                }
            )
        else:
            _emit(resolved.summary())
    except (BindingError, ContractError, OSError) as error:
        return _failed(error)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
