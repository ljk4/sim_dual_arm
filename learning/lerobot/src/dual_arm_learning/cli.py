"""Public command line for conversion, training, budget and model bundles."""

from __future__ import annotations

import argparse
import dataclasses
import json
import sys
from collections.abc import Sequence
from pathlib import Path

from .budget import BudgetError, BudgetLedger
from .bundle import BundleError, export_bundle, load_bundle
from .checkpoints import CheckpointError
from .config import ConfigError, load_dataset_config, load_training_config
from .dataset import ExportError, convert
from .sample_filter import (
    FilterError,
    build_filter_manifest,
    load_filter_mask,
    write_filter_manifest,
)
from .training import TrainingError, train, verify_dataset_manifest


def _default_root() -> Path:
    return Path(__file__).resolve().parents[4]


def _print(value: object) -> None:
    print(json.dumps(value, indent=2, sort_keys=True, default=str))


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--project-root", type=Path, default=_default_root())
    parser.add_argument("--debug", action="store_true", help="show a traceback on failure")
    sub = parser.add_subparsers(dest="command", required=True)

    convert_parser = sub.add_parser("convert", help="convert accepted 034 exports to 45/5 datasets")
    convert_parser.add_argument(
        "--config", type=Path, default=Path("learning/lerobot/configs/act_dataset.yaml")
    )
    convert_parser.add_argument("--select", type=Path, default=None, help="campaign select JSON")
    convert_parser.add_argument("--output-root", type=Path, default=None)
    convert_parser.add_argument(
        "--replace", action="store_true", help="replace our own incomplete output"
    )
    convert_parser.add_argument(
        "--synthetic",
        action="store_true",
        help="test fixture only; recorded as synthetic and never claims formal data",
    )

    train_parser = sub.add_parser("train", help="run ACT smoke/overfit/full training")
    train_parser.add_argument(
        "--config", type=Path, default=Path("learning/lerobot/configs/act_training.yaml")
    )
    train_parser.add_argument("--mode", choices=("smoke", "overfit", "full"), default=None)
    train_parser.add_argument("--resume-run", type=Path, default=None)
    train_parser.add_argument("--max-steps", type=int, default=None)
    train_parser.add_argument("--data-root", type=Path, default=None)
    train_parser.add_argument("--allow-synthetic", action="store_true")

    filter_parser = sub.add_parser(
        "filter-anchors", help="write an auditable static-anchor sampling mask"
    )
    filter_parser.add_argument("--config", type=Path, required=True)
    filter_parser.add_argument("--dataset-root", type=Path, required=True)
    filter_parser.add_argument("--select", type=Path, required=True)
    filter_parser.add_argument("--output", type=Path, required=True)

    verify_parser = sub.add_parser("verify-data", help="check 45/5 split, hashes and source files")
    verify_parser.add_argument(
        "--config", type=Path, default=Path("learning/lerobot/configs/act_training.yaml")
    )
    verify_parser.add_argument("--mode", choices=("smoke", "overfit", "full"), default=None)
    verify_parser.add_argument("--data-root", type=Path, default=None)
    verify_parser.add_argument("--allow-synthetic", action="store_true")

    budget_parser = sub.add_parser("budget-status", help="show the append-only training budget")
    budget_parser.add_argument(
        "--config", type=Path, default=Path("learning/lerobot/configs/act_training.yaml")
    )

    bundle_parser = sub.add_parser("bundle", help="export a safetensors-only offline ACT bundle")
    bundle_parser.add_argument(
        "--config", type=Path, default=Path("learning/lerobot/configs/act_training.yaml")
    )
    bundle_parser.add_argument("--mode", choices=("smoke", "overfit", "full"), default=None)
    bundle_parser.add_argument("--run", type=Path, required=True)
    bundle_parser.add_argument("--output", type=Path, required=True)
    bundle_parser.add_argument("--checkpoint", default="best.pt")
    bundle_parser.add_argument("--data-root", type=Path, default=None)
    bundle_parser.add_argument("--allow-synthetic", action="store_true")

    inspect_parser = sub.add_parser(
        "verify-bundle", help="verify and load a bundle without predicting"
    )
    inspect_parser.add_argument("--bundle", type=Path, required=True)
    inspect_parser.add_argument("--device", default="cpu")
    return parser


def _training_config(args: argparse.Namespace) -> object:
    config = load_training_config(
        args.project_root / args.config if not args.config.is_absolute() else args.config,
        args.project_root,
    )
    if getattr(args, "mode", None):
        config = dataclasses.replace(config, mode=args.mode)
    if getattr(args, "data_root", None):
        data_root = args.data_root
        if not data_root.is_absolute():
            data_root = args.project_root / data_root
        config = dataclasses.replace(config, data_root=data_root)
    return config


def main(argv: Sequence[str] | None = None) -> int:
    parser = build_parser()
    try:
        args = parser.parse_args(argv)
    except SystemExit as error:
        return int(error.code or 0)
    root = args.project_root.resolve()
    try:
        if args.command == "convert":
            config_path = args.config if args.config.is_absolute() else root / args.config
            config = load_dataset_config(config_path, root)
            changes: dict[str, object] = {}
            if args.select is not None:
                changes["select_path"] = (
                    args.select if args.select.is_absolute() else root / args.select
                )
            if args.output_root is not None:
                changes["output_root"] = (
                    args.output_root if args.output_root.is_absolute() else root / args.output_root
                )
            if changes:
                config = dataclasses.replace(config, **changes)  # type: ignore[arg-type]
            manifest = convert(
                select_path=config.select_path,
                config=config,
                config_path=config_path,
                replace=args.replace,
                synthetic=args.synthetic,
            )
            _print(
                {
                    "status": manifest["status"],
                    "synthetic": manifest["synthetic"],
                    "output_root": str(config.output_root),
                    "training_episodes": manifest["splits"]["training"]["episode_count"],
                    "validation_episodes": manifest["splits"]["validation"]["episode_count"],
                    "training_frames": manifest["splits"]["training"]["frame_count"],
                    "validation_frames": manifest["splits"]["validation"]["frame_count"],
                    "manifest": str(config.output_root / "manifest.json"),
                }
            )
            return 0
        if args.command == "train":
            config = _training_config(args)
            report = train(
                config,
                allow_synthetic=args.allow_synthetic,
                resume_run=args.resume_run,
                max_steps=args.max_steps,
            )
            _print(
                {
                    "status": report["reason"],
                    "run_id": report["run_id"],
                    "mode": report["mode"],
                    "steps": report["steps"],
                    "device": report["device"],
                    "best_val_l1": report["best_val_l1"],
                    "duration_seconds": report["duration_seconds"],
                    "run_dir": str(config.checkpoint.root / report["run_id"]),
                }
            )
            return 0
        if args.command == "filter-anchors":
            rule = args.config if args.config.is_absolute() else root / args.config
            dataset_root = (
                args.dataset_root if args.dataset_root.is_absolute() else root / args.dataset_root
            )
            select = args.select if args.select.is_absolute() else root / args.select
            output = args.output if args.output.is_absolute() else root / args.output
            document = build_filter_manifest(
                rule_path=rule, dataset_root=dataset_root, select_path=select
            )
            target = write_filter_manifest(document, output)
            mask = load_filter_mask(target)
            training = document["training"]
            _print(
                {
                    "status": "complete",
                    "output": str(target),
                    "filter_digest": mask.digest,
                    "mask_sha256": mask.file_sha256,
                    "training_source_frames": mask.source_frames,
                    "training_eligible_frames": len(mask.eligible_indices),
                    "training_excluded_frames": mask.excluded_frames,
                    "training_reduction_ratio": training["reduction_ratio"],
                    "validation_filtered": document["validation"]["filtered"],
                    "validation_frames": document["validation"]["kept_frames"],
                }
            )
            return 0
        if args.command == "verify-data":
            config = _training_config(args)
            audit = verify_dataset_manifest(
                config,
                allow_synthetic=args.allow_synthetic,
                require_full=config.mode == "full",
            )
            _print(dataclasses.asdict(audit))
            return 0
        if args.command == "budget-status":
            config = _training_config(args)
            ledger = BudgetLedger(
                config.budget.ledger, config.budget.lock, config.budget.limit_seconds
            )
            _print(ledger.status())
            return 0
        if args.command == "bundle":
            config_path = args.config if args.config.is_absolute() else root / args.config
            config = load_training_config(config_path, root)
            if args.data_root is not None:
                data_root = (
                    args.data_root if args.data_root.is_absolute() else root / args.data_root
                )
                config = dataclasses.replace(config, data_root=data_root)
            run = args.run if args.run.is_absolute() else root / args.run
            output = args.output if args.output.is_absolute() else root / args.output
            manifest = export_bundle(
                config=config,
                run_dir=run,
                output_dir=output,
                checkpoint_name=args.checkpoint,
                allow_synthetic=args.allow_synthetic,
            )
            _print(
                {
                    "status": "complete",
                    "bundle": str(output),
                    "bundle_digest": manifest["bundle_digest"],
                    "files": len(manifest["files"]),
                    "checkpoint_step": manifest["checkpoint"]["step"],
                    "synthetic": manifest["synthetic"],
                }
            )
            return 0
        if args.command == "verify-bundle":
            handle = load_bundle(args.bundle, device=args.device)
            _print(
                {
                    "status": "verified",
                    "bundle": str(handle.root),
                    "bundle_digest": handle.manifest.get("bundle_digest"),
                    "chunk_size": handle.manifest["chunk_size"],
                    "state_dim": handle.manifest["state_dim"],
                    "checkpoint_step": handle.manifest.get("checkpoint", {}).get("step"),
                    "deployment": handle.manifest.get("deployment"),
                }
            )
            return 0
        parser.error(f"unknown command {args.command!r}")
    except (
        ConfigError,
        ExportError,
        FilterError,
        TrainingError,
        BundleError,
        BudgetError,
        CheckpointError,
        OSError,
        ValueError,
    ) as error:
        if args.debug:
            raise
        print(f"learning: {type(error).__name__}: {error}", file=sys.stderr)
        return 1
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
