#!/usr/bin/env python3
"""Read-only scenario binding validator and viewer.

Examples:
  python3 shared/common/tools/scenario.py validate \
      --robot marvin_sim --task stack --stage collection
  python3 shared/common/tools/scenario.py show \
      --scenario marvin_sim/stack
  python3 shared/common/tools/scenario.py show \
      --binding shared/scenarios/marvin_sim/stack/binding.yaml

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
    resolve_binding_path,
)
from robot_contracts.models import ContractError  # noqa: E402


def _shared_root(value: str | None) -> Path:
    if value is not None:
        return Path(value)
    environment = os.environ.get("SHARED_ROOT")
    if environment:
        return Path(environment)
    return _LAYOUT_ROOT


def _add_selection(subparser: argparse.ArgumentParser) -> None:
    subparser.add_argument("--binding", help="path to binding.yaml")
    subparser.add_argument(
        "--scenario", help="scenario as <robot>/<task> below the shared scenarios tree"
    )
    subparser.add_argument("--robot", help="robot id below <shared-root>/scenarios")
    subparser.add_argument("--task", help="task id below <shared-root>/scenarios/<robot>")


def _binding_path(arguments: argparse.Namespace, shared_root: Path) -> Path:
    direct = arguments.binding
    scenario = arguments.scenario
    robot = arguments.robot
    task = arguments.task
    if direct is not None:
        if scenario is not None or robot is not None or task is not None:
            raise BindingError(
                "BINDING_REFERENCE",
                "--binding cannot be combined with --scenario/--robot/--task",
            )
        return Path(direct)
    if scenario is not None:
        if robot is not None or task is not None:
            raise BindingError(
                "BINDING_REFERENCE", "--scenario cannot be combined with --robot/--task"
            )
        parts = scenario.split("/")
        if len(parts) != 2 or not all(parts):
            raise BindingError(
                "BINDING_REFERENCE", f"--scenario {scenario!r}: expected <robot>/<task>"
            )
        robot, task = parts
    elif robot is None or task is None:
        raise BindingError(
            "BINDING_REFERENCE", "select a binding with --binding, --scenario or --robot/--task"
        )
    return resolve_binding_path(shared_root, robot, task)


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
    _add_selection(validate)
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
    _add_selection(show)
    return parser


def _emit(document: dict) -> None:
    json.dump(document, sys.stdout, indent=2, ensure_ascii=False, allow_nan=False)
    sys.stdout.write("\n")


def _failed(error: Exception) -> int:
    print(f"error: {error}", file=sys.stderr)
    return 2


def main(argv: list[str] | None = None) -> int:
    arguments = _parser().parse_args(argv)
    shared_root = _shared_root(arguments.shared_root)
    try:
        resolved: ResolvedBinding = load_binding(
            _binding_path(arguments, shared_root),
            shared_root=shared_root,
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
