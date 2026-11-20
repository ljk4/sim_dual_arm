#!/usr/bin/env python3
"""Read-only ACT evaluation report and three-view evidence builder.

The ledger is never modified: this tool reads the append-only attempt records,
per-attempt manifests/exports and ACT policy event logs, then writes a separate
report directory.  Every fixed test seed is listed, including missing, rejected,
timed-out and failed attempts.  A missing prediction/application record is
reported as an incomplete closed loop, never as an executed ACT rollout.

Per-seed evidence reconstruction lives in :mod:`evaluation_metrics` and preview
media generation in :mod:`evaluation_media`.
"""

from __future__ import annotations

import argparse
import json
import os
import sys
from pathlib import Path
from typing import Any

from robot_contracts.project_paths import project_path

from dual_arm_runtime.artifact_paths import debug_root
from dual_arm_runtime.offline import campaign
from dual_arm_runtime.offline.evaluation_media import (  # noqa: E402
    keyframes,
    media_for,
)
from dual_arm_runtime.offline.evaluation_metrics import (  # noqa: E402
    metric_for_seed as _metric_for_seed,  # noqa: F401 - re-export used by tests
)

ROOT = Path(os.environ.get("PROJECT_ROOT") or Path(__file__).resolve().parents[6])


def _markdown(metrics: list[dict[str, Any]], summary: dict[str, Any]) -> str:
    lines = [
        "# ACT evaluation report",
        "",
        "Read-only reconstruction from the append-only campaign ledger.  All fixed "
        "test seeds are counted; missing/rejected/timeout rows are not removed.",
        "",
        "`first label` is the 20 Hz exported label sampling time, not a 2 ms physical "
        "application instant; `first started` is the recorded block start.",
        "",
        f"- seeds: `{summary['seed_first']}..{summary['seed_last']}`",
        f"- denominator: {summary['denominator']}",
        f"- reservations: {summary['reserved']}",
        f"- terminal attempts: {summary['terminal']}",
        f"- credible ACT executions: {summary['act_executed']}",
        f"- closed loop: {summary['closed_loop']}",
        f"- physical task success: {summary['task_success']}",
        f"- physical success (ACT + task): {summary['success']}",
        f"- full run success (also runtime/recording/quality/launch): {summary['run_success']}",
        f"- quality qualified: {summary['quality_qualified']}",
        f"- ACT evidence incomplete: {summary['act_evidence_incomplete']}",
        f"- task evidence incomplete: {summary['task_evidence_incomplete']}",
        f"- errors/issues: {summary['issues']}",
        "",
        "| seed | attempt | task | task eval | ACT | closed | runtime | rec | launch | "
        "quality | physical success | run success | first started (s) | first label (s) | "
        "blocks | task sim (s) | exec span (s) | wall (s) | P50/P95/max (ms) | "
        "VRAM (MiB) | failure | evidence |",
        "| ---: | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | "
        "---: | ---: | ---: | ---: | ---: | ---: | --- | ---: | --- | --- | --- |",
    ]
    for metric in metrics:
        first = metric["first_started_sim_time_ns"]
        first_s = "" if first is None else f"{first / 1e9:.3f}"
        label = metric["first_exported_label_sim_time_ns"]
        label_s = "" if label is None else f"{label / 1e9:.3f}"
        latency = "/".join(
            "" if metric.get(key) is None else str(metric.get(key))
            for key in ("prediction_p50_ms", "prediction_p95_ms", "prediction_max_ms")
        )
        evidence = metric.get("preview_video") or metric.get("recording_path") or ""
        lines.append(
            "| {seed} | {attempt} | {task} | {eval} | {act} | {closed} | {runtime} | "
            "{rec} | {launch} | {quality} | {physical} | {run} | {first} | {label} | "
            "{blocks} | {task_sim} | {exec} | {wall} | {latency} | {vram} | "
            "{failure} | {evidence} |".format(
                seed=metric["seed"],
                attempt=metric.get("attempt_id") or "-",
                task=metric.get("task_outcome") or "-",
                eval="yes" if metric.get("task_evaluated") else "no",
                act="yes" if metric.get("act_executed") else "no",
                closed="yes" if metric.get("closed_loop") else "no",
                runtime=metric.get("runtime_outcome") or "-",
                rec="yes" if metric.get("recording_complete") else "no",
                launch="yes" if metric.get("launch_ok") else "no",
                quality="yes" if metric.get("quality_qualified") else "no",
                physical="yes" if metric.get("success") else "no",
                run="yes" if metric.get("run_success") else "no",
                first=first_s or "-",
                label=label_s or "-",
                blocks=metric.get("command_blocks", 0),
                task_sim=""
                if metric.get("task_duration_sim_seconds") is None
                else metric["task_duration_sim_seconds"],
                exec=""
                if metric.get("execution_span_seconds") is None
                else metric["execution_span_seconds"],
                wall="" if metric.get("wall_seconds") is None else metric["wall_seconds"],
                latency=latency or "-",
                vram="" if metric.get("vram_peak_mib") is None else metric["vram_peak_mib"],
                failure=metric.get("failure_type") or "ok",
                evidence=evidence,
            )
        )
    return "\n".join(lines) + "\n"


def _metrics_for_report(
    project_root: Path,
    config: dict[str, Any],
    records: list[dict[str, Any]],
    purposes: tuple[str, ...],
) -> tuple[list[dict[str, Any]], list[int], int, int]:
    terminal = campaign._finished(records)
    seed_first = int(config["budget"]["test_seed_first"])
    seed_last = int(config["budget"]["test_seed_last"])
    by_seed: dict[int, dict[str, Any]] = {}
    duplicates: list[int] = []
    for purpose in purposes:
        for record in campaign._attempts(records, purpose):
            seed = int(record.get("seed", -1))
            if seed in by_seed:
                duplicates.append(seed)
                continue
            by_seed[seed] = record
    metrics = [
        _metric_for_seed(
            project_root,
            config,
            seed,
            by_seed.get(seed),
            terminal.get(str(by_seed[seed]["attempt_id"])) if seed in by_seed else None,
        )
        for seed in range(seed_first, seed_last + 1)
    ]
    return metrics, duplicates, seed_first, seed_last


def _generate_media(metrics: list[dict[str, Any]], output: Path) -> None:
    for metric in metrics:
        if metric.get("closed_loop"):
            media_for(metric, output)
        elif metric.get("export"):
            metric["preview_keyframes"] = keyframes(
                Path(metric["export"]), output / "preview", f"seed-{metric['seed']:05d}"
            )


def _summary(
    metrics: list[dict[str, Any]], duplicates: list[int], seed_first: int, seed_last: int
) -> dict[str, Any]:
    model_digests = sorted(
        {str(item["bundle_digest"]) for item in metrics if item.get("bundle_digest")}
    )
    checkpoint_steps = sorted(
        {int(item["checkpoint_step"]) for item in metrics if item.get("checkpoint_step")}
    )
    issues = {
        item["failure_type"]
        for item in metrics
        if item["failure_type"] in {"MISSING_RESERVATION", "MISSING_ATTEMPT_DIRECTORY"}
    }
    if len(model_digests) > 1:
        issues.add("MULTIPLE_MODEL_DIGESTS")
    if len(checkpoint_steps) > 1:
        issues.add("MULTIPLE_CHECKPOINT_STEPS")
    return {
        "seed_first": seed_first,
        "seed_last": seed_last,
        "denominator": len(metrics),
        "reserved": sum(1 for item in metrics if item["reserved"]),
        "terminal": sum(1 for item in metrics if item["terminal"]),
        "act_executed": sum(1 for item in metrics if item["act_executed"]),
        "closed_loop": sum(1 for item in metrics if item["closed_loop"]),
        "task_success": sum(1 for item in metrics if item.get("task_success")),
        "success": sum(1 for item in metrics if item.get("success")),
        "run_success": sum(1 for item in metrics if item.get("run_success")),
        "quality_qualified": sum(1 for item in metrics if item.get("quality_qualified")),
        "act_evidence_incomplete": sum(1 for item in metrics if not item["act_executed"]),
        "task_evidence_incomplete": sum(
            1 for item in metrics if not item.get("task_evaluated") or not item.get("task_evidence")
        ),
        "duplicate_seeds": sorted(set(duplicates)),
        "model_digests": model_digests,
        "checkpoint_steps": checkpoint_steps,
        "issues": sorted(issues),
    }


def build_report(
    *,
    project_root: Path,
    config_path: Path,
    purposes: tuple[str, ...] = ("test", "evaluation"),
    output: Path,
    generate_media: bool = True,
) -> dict[str, Any]:
    config = campaign._load(config_path)
    paths = config["paths"]
    ledger = campaign.Ledger(
        project_path(paths["ledger"], project_root), project_path(paths["lock"], project_root)
    )
    with ledger.locked():
        records = ledger.records()
    metrics, duplicates, seed_first, seed_last = _metrics_for_report(
        project_root, config, records, purposes
    )
    if generate_media:
        _generate_media(metrics, output)
    summary = _summary(metrics, duplicates, seed_first, seed_last)
    document = {
        "ledger": str(project_path(paths["ledger"], project_root)),
        "purposes": list(purposes),
        "summary": summary,
        "attempts": metrics,
        "notes": [
            "fixed test seeds are never replaced or re-run to select a better result",
            "a missing prediction/application record is reported as an incomplete closed loop",
            "physical success requires credible ACT execution plus independent physical task facts",
            "quality is reported separately and does not by itself define physical task success",
            "previews are generated from recorded RGB only and never fed back to policy/training",
        ],
    }
    output.mkdir(parents=True, exist_ok=True)
    (output / "evaluation.json").write_text(
        json.dumps(document, indent=2, allow_nan=False) + "\n", encoding="utf-8"
    )
    (output / "report.md").write_text(_markdown(metrics, summary), encoding="utf-8")
    return document


def parse_arguments(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--project-root", type=Path, default=ROOT)
    parser.add_argument(
        "--config",
        type=Path,
        default=None,
        help="direct campaign recipe (conflicts with --robot/--task)",
    )
    parser.add_argument("--robot")
    parser.add_argument("--task")
    parser.add_argument("--shared-root", type=Path)
    parser.add_argument("--purpose", choices=("all", "test", "evaluation"), default="all")
    parser.add_argument("--output", type=Path, default=None)
    parser.add_argument("--no-media", action="store_true")
    args = parser.parse_args(argv)
    if args.robot is not None and args.config is not None:
        parser.error("--robot/--task cannot be combined with --config")
    if args.robot is None and args.config is None:
        parser.error("one of --robot/--task or --config is required")
    if args.robot is not None and args.task is None:
        parser.error("--robot requires --task")
    if args.task is not None and args.robot is None:
        parser.error("--task requires --robot")
    if args.shared_root is not None and args.robot is None:
        parser.error("--shared-root requires --robot/--task")
    if args.output is None:
        args.output = debug_root(Path("var"), args.robot, args.task) / "evaluation"
    return args


def main(argv: list[str] | None = None) -> int:
    from dual_arm_runtime.offline.scenario_selection import campaign_config_path

    args = parse_arguments(argv)
    project_root = args.project_root.resolve()
    output = args.output if args.output.is_absolute() else project_path(args.output, project_root)
    try:
        document = build_report(
            project_root=project_root,
            config_path=campaign_config_path(
                campaign_config=args.config,
                robot=args.robot,
                task=args.task,
                shared_root=args.shared_root,
                project_root=project_root,
                default=None,
            ),
            purposes=(("test", "evaluation") if args.purpose == "all" else (args.purpose,)),
            output=output.resolve(),
            generate_media=not args.no_media,
        )
    except Exception as error:  # noqa: BLE001 - report failure must be visible
        print(f"evaluate: {type(error).__name__}: {error}", file=sys.stderr)
        return 1
    print(json.dumps(document["summary"], indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
