"""Public command line for conversion, training, budget and model bundles."""

from __future__ import annotations

import argparse
import contextlib
import dataclasses
import json
import os
import sys
import time
from collections.abc import Sequence
from pathlib import Path

from robot_contracts.project_paths import project_path

from .budget import BudgetError, BudgetLedger
from .bundle import BundleError, export_bundle, load_bundle
from .checkpoints import CheckpointError
from .config import ConfigError, load_dataset_config, load_training_config
from .console_progress import OperatorConsole
from .dataset import ExportError, convert
from .sample_filter import (
    FilterError,
    build_filter_manifest,
    load_filter_mask,
    write_filter_manifest,
)
from .scenario import (
    ScenarioSelectionError,
    check_dataset_recipe_consistency,
    check_profile_consistency,
    select_scenario,
)
from .training import TrainingError, train, verify_dataset_manifest


def _default_root() -> Path:
    root = os.environ.get("PROJECT_ROOT")
    return Path(root) if root else Path(__file__).resolve().parents[4]


def _print(value: object) -> None:
    print(json.dumps(value, indent=2, sort_keys=True, default=str))


def _add_selection_arguments(parser: argparse.ArgumentParser) -> None:
    """Every command needs one explicit recipe source; there is no default."""
    selection = parser.add_mutually_exclusive_group(required=True)
    selection.add_argument(
        "--config", type=Path, default=None, help="direct dataset/training recipe path"
    )
    selection.add_argument(
        "--scenario", help="scenario id under <shared-root>/scenarios/<id>/binding.yaml"
    )
    parser.add_argument(
        "--shared-root", type=Path, default=None, help="shared tree containing scenarios/"
    )
    parser.add_argument(
        "--set",
        dest="overrides",
        action="append",
        default=[],
        metavar="PATH=VALUE",
        help="override one existing recipe path (repeatable)",
    )


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--project-root", type=Path, default=_default_root())
    parser.add_argument("--debug", action="store_true", help="show a traceback on failure")
    sub = parser.add_subparsers(dest="command", required=True)

    convert_parser = sub.add_parser(
        "convert", help="convert accepted exports to independent train/validation datasets"
    )
    _add_selection_arguments(convert_parser)
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
    _add_selection_arguments(train_parser)
    train_parser.add_argument("--mode", choices=("smoke", "overfit", "full"), default=None)
    train_parser.add_argument("--resume-run", type=Path, default=None)
    train_parser.add_argument("--max-steps", type=int, default=None)
    train_parser.add_argument("--data-root", type=Path, default=None)
    train_parser.add_argument("--allow-synthetic", action="store_true")

    for command_parser in (convert_parser, train_parser):
        command_parser.add_argument(
            "--progress",
            action=argparse.BooleanOptionalAction,
            default=False,
            help="show concise operator progress and send detailed output to a log file",
        )
        command_parser.add_argument("--log-file", type=Path, help="detail log with --progress")

    filter_parser = sub.add_parser(
        "filter-anchors", help="write an auditable static-anchor sampling mask"
    )
    filter_parser.add_argument("--config", type=Path, required=True)
    filter_parser.add_argument("--dataset-root", type=Path, required=True)
    filter_parser.add_argument("--select", type=Path, required=True)
    filter_parser.add_argument("--output", type=Path, required=True)

    verify_parser = sub.add_parser(
        "verify-data", help="check the configured split, hashes and source files"
    )
    _add_selection_arguments(verify_parser)
    verify_parser.add_argument("--mode", choices=("smoke", "overfit", "full"), default=None)
    verify_parser.add_argument("--data-root", type=Path, default=None)
    verify_parser.add_argument("--allow-synthetic", action="store_true")

    budget_parser = sub.add_parser("budget-status", help="show the append-only training budget")
    _add_selection_arguments(budget_parser)

    bundle_parser = sub.add_parser("bundle", help="export a safetensors-only offline ACT bundle")
    _add_selection_arguments(bundle_parser)
    bundle_parser.add_argument(
        "--progress",
        action=argparse.BooleanOptionalAction,
        default=False,
        help="print resolved config paths and parameters before exporting",
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


def _training_config(args: argparse.Namespace, config_path: Path) -> object:
    config = load_training_config(config_path, args.project_root, overrides=args.overrides)
    if getattr(args, "mode", None):
        config = config.model_copy(update={"mode": args.mode})
    if getattr(args, "data_root", None):
        data_root = project_path(args.data_root, args.project_root)
        config = config.model_copy(update={"data_root": data_root})
    return config


def _resolve(path: Path, root: Path) -> Path:
    return project_path(path, root)


def _scenario_logs_root(args: argparse.Namespace, root: Path) -> Path:
    segment = args.scenario or "_direct"
    return project_path(Path("var") / segment / "logs", root)


def _learning_selection(args: argparse.Namespace, root: Path, *, recipe_field: str):
    """Return the real recipe path and the scenario provenance (or None)."""
    if args.scenario is not None:
        selection = select_scenario(
            scenario=args.scenario,
            recipe_field=recipe_field,
            stage="learning",
            project_root=root,
            shared_root=args.shared_root,
        )
        return selection.recipe_path, selection
    if args.shared_root is not None:
        raise ScenarioSelectionError("--shared-root requires --scenario")
    return _resolve(args.config, root), None


def _check_selection(
    selection, *, profile_path: Path, dataset_config_path: Path | None = None
) -> None:
    if selection is None:
        return
    check_profile_consistency(selection, profile_path)
    if dataset_config_path is not None:
        check_dataset_recipe_consistency(selection, dataset_config_path)


def _scenario_manifest(selection, *, dataset_recipe: bool = False) -> dict | None:
    if selection is None:
        return None
    return selection.summary(include_dataset_recipe=dataset_recipe)


def _training_overrides(args: argparse.Namespace, root: Path) -> dict[str, object]:
    overrides: dict[str, object] = {}
    mode = getattr(args, "mode", None)
    if mode is not None:
        overrides["mode"] = {"value": mode, "source": "argument", "target": "mode"}
    max_steps = getattr(args, "max_steps", None)
    if max_steps is not None:
        overrides["max_steps"] = {"value": max_steps, "source": "argument"}
    resume_run = getattr(args, "resume_run", None)
    if resume_run is not None:
        overrides["resume_run"] = {
            "value": str(_resolve(resume_run, root)),
            "source": "argument",
        }
    data_root = getattr(args, "data_root", None)
    if data_root is not None:
        overrides["data_root"] = {
            "value": str(_resolve(data_root, root)),
            "source": "argument",
            "target": "data.root",
        }
    if getattr(args, "allow_synthetic", False):
        overrides["allow_synthetic"] = {"value": True, "source": "argument"}
    if getattr(args, "overrides", None):
        overrides["set"] = {"value": list(args.overrides), "source": "argument"}
    return overrides


def _operator_console(args, title, settings, default_log):
    if not args.progress:
        return contextlib.nullcontext(None)
    path = args.log_file or default_log
    return OperatorConsole(_resolve(path, args.project_root), title, settings)


def _print_effective_config(
    title: str,
    config_path: Path,
    selection: object,
    config: object,
) -> None:
    print(f"=== {title} ===", flush=True)
    if selection is not None:
        print(f"方案 (scenario): {selection.scenario_id}", flush=True)
        print(f"Binding 文件:    {selection.resolved.path}", flush=True)
    print(f"配置文件路径:    {config_path}", flush=True)
    print("--- 详细参数 ---", flush=True)
    print(json.dumps(config.model_dump(mode="json"), indent=2, ensure_ascii=False), flush=True)
    print("=" * (len(title) + 8), flush=True)


def _command_convert(args: argparse.Namespace, root: Path) -> int:
    config_path, selection = _learning_selection(args, root, recipe_field="dataset")
    config = load_dataset_config(config_path, root, overrides=args.overrides)
    _check_selection(selection, profile_path=config.profile_path)
    changes: dict[str, object] = {}
    overrides: dict[str, object] = {}
    if args.select is not None:
        changes["select_path"] = _resolve(args.select, root)
        overrides["select"] = {"value": str(changes["select_path"]), "source": "argument"}
    if args.output_root is not None:
        changes["output_root"] = _resolve(args.output_root, root)
        overrides["output_root"] = {"value": str(changes["output_root"]), "source": "argument"}
    if args.replace:
        overrides["replace"] = {"value": True, "source": "argument"}
    if args.synthetic:
        overrides["synthetic"] = {"value": True, "source": "argument"}
    if args.overrides:
        overrides["set"] = {"value": list(args.overrides), "source": "argument"}
    if changes:
        config = config.model_copy(update=changes)
    settings = [
        f"清单：{config.select_path}",
        f"输出：{config.output_root}",
        f"划分：{config.expected_training_episodes} 条训练 / "
        f"{config.expected_validation_episodes} 条验证；{config.fps} Hz；"
        f"{len(config.camera_names)} 路 {config.camera_width}×{config.camera_height} RGB",
    ]
    if args.progress:
        _print_effective_config("数据转换配置与详细参数", config_path, selection, config)
    default_log = _scenario_logs_root(args, root) / "convert" / f"convert-{time.time_ns()}.log"
    with _operator_console(args, "数据转换配置", settings, default_log) as console:
        manifest = convert(
            select_path=config.select_path,
            config=config,
            config_path=config_path,
            replace=args.replace,
            synthetic=args.synthetic,
            progress=None if console is None else console.progress,
            selection=_scenario_manifest(selection),
            overrides=overrides or None,
        )
    if args.progress:
        splits = manifest["splits"]
        print(
            f"转换完成：训练 {splits['training']['episode_count']} 条 / "
            f"{splits['training']['frame_count']} 帧；"
            f"验证 {splits['validation']['episode_count']} 条 / "
            f"{splits['validation']['frame_count']} 帧"
        )
        print(f"数据集：{config.output_root}")
        return 0
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


def _command_train(args: argparse.Namespace, root: Path) -> int:
    config_path, selection = _learning_selection(args, root, recipe_field="training")
    config = _training_config(args, config_path)
    _check_selection(
        selection,
        profile_path=config.dataset.profile_path,
        dataset_config_path=config.dataset_config_path,
    )
    steps = (
        config.mode_spec.steps
        if args.max_steps is None
        else min(config.mode_spec.steps, args.max_steps)
    )
    settings = [
        f"模式：{config.mode}；设备：{config.device}；目标：{steps} step；"
        f"batch：{config.batch_size}",
        f"学习率：{config.lr:g}；AMP：{config.amp_dtype if config.use_amp else '关闭'}；"
        f"数据 worker：{config.num_workers}",
        f"数据集：{config.data_root}",
        f"验证：每 {config.val_every_steps} step；检查点：每 {config.save_every_steps} step",
    ]
    if args.progress:
        _print_effective_config("ACT 训练配置与详细参数", config_path, selection, config)
    default_log = config.logging.root / "_console" / f"train-{time.time_ns()}.log"
    with _operator_console(args, "ACT 训练配置", settings, default_log) as console:
        if console is not None:
            console.progress.phase("校验训练数据")
        report = train(
            config,
            allow_synthetic=args.allow_synthetic,
            resume_run=None if args.resume_run is None else _resolve(args.resume_run, root),
            max_steps=args.max_steps,
            progress=None if console is None else console.progress,
            selection=_scenario_manifest(selection, dataset_recipe=True),
            overrides=_training_overrides(args, root) or None,
        )
    if args.progress:
        print(
            f"训练结束：{report['reason']}；{report['steps']}/{steps} step；"
            f"best_val_l1={report['best_val_l1']}；用时 {report['duration_seconds']:.1f}s"
        )
        print(f"run_dir: {config.checkpoint.root / report['run_id']}")
        return 0
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


def _command_filter(args: argparse.Namespace, root: Path) -> int:
    document = build_filter_manifest(
        rule_path=_resolve(args.config, root),
        dataset_root=_resolve(args.dataset_root, root),
        select_path=_resolve(args.select, root),
    )
    target = write_filter_manifest(document, _resolve(args.output, root))
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


def _command_verify_data(args: argparse.Namespace, root: Path) -> int:
    config_path, selection = _learning_selection(args, root, recipe_field="training")
    config = _training_config(args, config_path)
    _check_selection(
        selection,
        profile_path=config.dataset.profile_path,
        dataset_config_path=config.dataset_config_path,
    )
    audit = verify_dataset_manifest(
        config,
        allow_synthetic=args.allow_synthetic,
        require_full=config.mode == "full",
    )
    _print(dataclasses.asdict(audit))
    return 0


def _command_budget(args: argparse.Namespace, root: Path) -> int:
    config_path, selection = _learning_selection(args, root, recipe_field="training")
    config = _training_config(args, config_path)
    _check_selection(
        selection,
        profile_path=config.dataset.profile_path,
        dataset_config_path=config.dataset_config_path,
    )
    ledger = BudgetLedger(config.budget.ledger, config.budget.lock, config.budget.limit_seconds)
    _print(ledger.status())
    return 0


def _command_bundle(args: argparse.Namespace, root: Path) -> int:
    config_path, selection = _learning_selection(args, root, recipe_field="training")
    config = _training_config(args, config_path)
    _check_selection(
        selection,
        profile_path=config.dataset.profile_path,
        dataset_config_path=config.dataset_config_path,
    )
    run = _resolve(args.run, root)
    output = _resolve(args.output, root)
    if args.progress:
        _print_effective_config("Bundle 导出配置与详细参数", config_path, selection, config)
        print(f"训练目录 (run):  {run}", flush=True)
        print(f"导出目录 (out):  {output}", flush=True)
    manifest = export_bundle(
        config=config,
        run_dir=run,
        output_dir=output,
        checkpoint_name=args.checkpoint,
        allow_synthetic=args.allow_synthetic,
        selection=_scenario_manifest(selection, dataset_recipe=True),
        overrides=_training_overrides(args, root) or None,
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


def _command_verify_bundle(args: argparse.Namespace, root: Path) -> int:
    del root
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


_COMMANDS = {
    "convert": _command_convert,
    "train": _command_train,
    "filter-anchors": _command_filter,
    "verify-data": _command_verify_data,
    "budget-status": _command_budget,
    "bundle": _command_bundle,
    "verify-bundle": _command_verify_bundle,
}


def main(argv: Sequence[str] | None = None) -> int:
    parser = build_parser()
    try:
        args = parser.parse_args(argv)
    except SystemExit as error:
        return int(error.code or 0)
    root = args.project_root.resolve()
    try:
        command = _COMMANDS.get(args.command)
        if command is None:
            parser.error(f"unknown command {args.command!r}")
        return command(args, root)
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
    except Exception as error:
        if args.debug or not getattr(args, "progress", False):
            raise
        print(f"learning: {type(error).__name__}: {error}", file=sys.stderr)
        return 1
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
