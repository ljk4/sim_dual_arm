#!/usr/bin/env python3
"""Run one already-reserved task through the owned Compose stack and export.

Campaign owns attempts and budgets. This module owns generated configuration,
process exit evidence and the read-only ROS export container. Physics, task
judgement, continuous execution and MCAP remain in their existing components.

ACT is selected explicitly: the generated runtime config uses the real bundle
digests and the independent ``act-policy`` overlay; the fake policy overlay is
never started.  Teacher mode needs no Torch or learning mount.
"""

from __future__ import annotations

import argparse
import dataclasses
import hashlib
import json
import os
import shutil
import statistics
import subprocess
import sys
import uuid
from pathlib import Path
from typing import Any

import yaml

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "tools/run"))
sys.path.insert(0, str(ROOT / "components/learning/src"))
sys.path.insert(0, str(ROOT / "libraries/run_logging/src"))
import native_backend  # noqa: E402
import native_policy  # noqa: E402
from dual_arm_learning.act_contract import (  # noqa: E402
    ActContract,
    ActContractError,
    ActSettings,
    load_act_settings,
    resolve_act_contract,
)


def write_json(path: Path, value: object) -> None:
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(json.dumps(value, indent=2, allow_nan=False) + "\n")
    temporary.replace(path)


def container_path(path: Path) -> str:
    return "/workspace/" + path.resolve().relative_to(ROOT).as_posix()


def _act_overrides(
    settings: ActSettings, args: argparse.Namespace
) -> ActSettings:
    changes: dict[str, Any] = {}
    for name in ("bundle", "profile", "observation_schema", "action_schema"):
        value = getattr(args, name, None)
        if value is not None:
            changes[name] = Path(value).resolve()
    if getattr(args, "device", None):
        changes["device"] = args.device
    return dataclasses.replace(settings, **changes) if changes else settings


def generate_act_config(
    template: Path,
    runtime: dict[str, Any],
    settings: ActSettings,
    contract: ActContract,
    generated: Path,
) -> Path:
    """Snapshot the real paths and derived contract next to generated runtime."""
    document = yaml.safe_load(template.read_text())
    model = document.get("model")
    if not isinstance(model, dict):
        raise ActContractError("ACT policy template has no model object")
    model["bundle"] = "/opt/act-bundle"
    model["profile"] = runtime["robot_profile"]
    model["observation_schema"] = runtime["observation_schema"]
    model["action_schema"] = runtime["action_schema"]
    model["device"] = settings.device
    model["require_formal"] = settings.require_formal
    service = document.get("service")
    if not isinstance(service, dict):
        raise ActContractError("ACT policy template has no service object")
    service["task_id"] = settings.task_id
    service["task_text"] = settings.task_text
    path = generated / "act_policy.yaml"
    path.write_text(yaml.safe_dump(document, sort_keys=False))
    write_json(
        generated / "act_contract.json",
        {
            "schema_version": 1,
            "contract": contract.summary(),
            "config": {
                "path": str(template.resolve()),
                "sha256": hashlib.sha256(template.read_bytes()).hexdigest(),
            },
            "generated": {
                "path": str(path),
                "sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
            },
        },
    )
    return path


def generate_inputs(
    recipe: Path,
    seed: int,
    run_id: str,
    *,
    settings: ActSettings | None = None,
    contract: ActContract | None = None,
    act_template: Path | None = None,
    backend_template: Path | None = None,
    generated_root: Path | None = None,
) -> tuple[Path, Path, Path | None, str]:
    """Resolve source-relative paths before moving a configuration file."""
    runtime = yaml.safe_load(recipe.read_text())
    source = runtime["action_source"]
    backend_source = backend_template or (
        ROOT
        / "configs/simulation"
        / ("backend_stack_teacher.yaml" if source == "teacher" else "backend_stack_inference.yaml")
    )
    backend = yaml.safe_load(backend_source.read_text())
    runtime_follower = runtime.get("follower_constraints")
    backend_follower = backend["robot"].get("follower")
    if runtime_follower and backend_follower:
        runtime_path = (recipe.parent / runtime_follower).resolve()
        backend_path = (backend_source.parent / backend_follower).resolve()
        if runtime_path != backend_path:
            raise ValueError("runtime and backend must use the same follower configuration")
    for field in ("profile", "schemas", "scene", "control", "stream", "follower", "task"):
        if backend["robot"].get(field):
            backend["robot"][field] = container_path(
                backend_source.parent / backend["robot"][field]
            )
    for field in (
        "robot_profile",
        "observation_schema",
        "action_schema",
        "control_constraints",
        "follower_constraints",
    ):
        if runtime.get(field):
            runtime[field] = container_path(recipe.parent / runtime[field])
    runtime["reset"] = {"enabled": True, "seed": seed}
    backend["run"]["seed"] = seed
    if settings is not None:
        if source != "policy":
            raise ActContractError("ACT deployment requires action_source: policy")
        if backend["robot"].get("task_mode") != "inference":
            raise ActContractError("ACT deployment requires backend task_mode: inference")
        if contract is None:
            raise ActContractError("ACT deployment requires a verified contract")
        policy = runtime.get("policy")
        if not isinstance(policy, dict):
            raise ActContractError("ACT runtime recipe has no policy object")
        policy["endpoint"] = f"act-policy:{settings.port}"
        policy["model_bundle_digest"] = contract.bundle_digest
        policy["processor_digest"] = contract.processor_digest
        policy["allow_test_policy"] = False
        policy["expected_test_only"] = False
        policy["task_id"] = settings.task_id
        policy["task_text"] = settings.task_text
        policy["predict_deadline_s"] = settings.predict_deadline_s
        policy["open_deadline_s"] = settings.open_deadline_s
        policy["close_deadline_s"] = settings.close_deadline_s
        policy["max_message_bytes"] = settings.max_message_bytes
        policy["connect_timeout_s"] = max(float(policy.get("connect_timeout_s", 2.0)), 5.0)
        runtime["robot_profile"] = container_path(settings.profile)
        runtime["observation_schema"] = container_path(settings.observation_schema)
        runtime["action_schema"] = container_path(settings.action_schema)
    base = generated_root or (ROOT / ".build/generated/stack-inputs")
    generated = base / run_id
    generated.mkdir(parents=True, exist_ok=False)
    paths = (generated / "runtime.yaml", generated / "backend.yaml")
    for path, document in zip(paths, (runtime, backend), strict=True):
        path.write_text(yaml.safe_dump(document, sort_keys=False))
    act_config = None
    if settings is not None and contract is not None:
        template = act_template or (ROOT / "configs/learning/act_policy.yaml")
        act_config = generate_act_config(template, runtime, settings, contract, generated)
    return *paths, act_config, source


def _read_events(directory: Path) -> list[dict[str, Any]]:
    result: list[dict[str, Any]] = []
    if not directory.is_dir():
        return result
    for path in sorted(directory.glob("events.*.jsonl")):
        try:
            lines = path.read_text(encoding="utf-8", errors="replace").splitlines()
        except OSError:
            continue
        for line in lines:
            text = line.strip()
            if not text.startswith("{"):
                continue
            try:
                event = json.loads(text)
            except json.JSONDecodeError:
                continue
            if isinstance(event, dict):
                result.append(event)
    return result


def policy_metrics(directory: Path) -> dict[str, Any] | None:
    events = _read_events(directory)
    if not events:
        return None
    latencies: list[int] = []
    model_latencies: list[int] = []
    vram: list[float] = []
    rejects = 0
    ready = False
    for event in events:
        name = event.get("event")
        fields = event.get("fields")
        if not isinstance(fields, dict):
            fields = {}
        if name == "act.model.ready" or name == "policy.ready":
            ready = True
        if name == "act.predict.finished":
            value = fields.get("latency_ns")
            if isinstance(value, int) and value >= 0:
                latencies.append(value)
            value = fields.get("model_latency_ns")
            if isinstance(value, int) and value >= 0:
                model_latencies.append(value)
            value = fields.get("vram_peak_mib")
            if isinstance(value, int | float) and value >= 0:
                vram.append(float(value))
        elif name in {"act.predict.rejected", "policy.rpc.rejected"}:
            rejects += 1
    if not latencies:
        return {
            "ready": ready,
            "predictions": 0,
            "rejected": rejects,
            "latency_ms": None,
            "vram_peak_mib": max(vram) if vram else None,
        }

    def percentile(values: list[int], fraction: float) -> float:
        ordered = sorted(values)
        position = min(len(ordered) - 1, max(0, round((len(ordered) - 1) * fraction)))
        return ordered[position] / 1_000_000

    return {
        "ready": ready,
        "predictions": len(latencies),
        "rejected": rejects,
        "latency_ms": {
            "p50": round(percentile(latencies, 0.50), 3),
            "p95": round(percentile(latencies, 0.95), 3),
            "max": round(max(latencies) / 1_000_000, 3),
            "mean": round(statistics.fmean(latencies) / 1_000_000, 3),
        },
        "model_latency_ms": {
            "p50": round(percentile(model_latencies, 0.50), 3),
            "p95": round(percentile(model_latencies, 0.95), 3),
            "max": round(max(model_latencies) / 1_000_000, 3),
        }
        if model_latencies
        else None,
        "vram_peak_mib": max(vram) if vram else None,
    }


def export_episode(
    episode: Path,
    stack_root: Path,
    output: Path,
    build_root: Path,
    policy_sdk: Path,
    image: str,
    run_id: str,
) -> int:
    name = "stack-export-" + uuid.uuid4().hex[:12]
    command = [
        "docker",
        "run",
        "--rm",
        "--name",
        name,
        "--network",
        "none",
        "--user",
        f"{os.getuid()}:{os.getgid()}",
        "--mount",
        f"type=bind,source={ROOT},target=/workspace,readonly",
        "--mount",
        f"type=bind,source={build_root},target=/opt/ros-dev,readonly",
        "--mount",
        f"type=bind,source={policy_sdk},target=/policy-sdk,readonly",
        "--mount",
        f"type=bind,source={stack_root},target=/data",
        "-e",
        "DEV_REPO_ROOT=/workspace",
        "-e",
        "DEV_ROS_ROOT=/opt/ros-dev",
        "-e",
        "PYTHONDONTWRITEBYTECODE=1",
        "-e",
        "PYTHONPATH=/policy-sdk:/workspace/components/learning/src",
        image,
        "python3",
        "/workspace/tools/run/stack_export.py",
        "--episode-dir",
        "/data/" + episode.relative_to(stack_root).as_posix(),
        "--out",
        "/data/export",
        "--campaign-config",
        "/data/campaign.yaml",
        "--runtime-log",
        f"/data/runs/{run_id}",
        "--backend-log",
        f"/data/{run_id}/log",
    ]
    try:
        with (
            (output / "export.stdout.txt").open("w") as stdout,
            (output / "export.stderr.txt").open("w") as stderr,
        ):
            return subprocess.run(command, stdout=stdout, stderr=stderr, timeout=120).returncode
    finally:
        subprocess.run(["docker", "rm", "-f", name], capture_output=True, timeout=15)


def campaign_alignment_error(
    *,
    campaign_mode: str,
    campaign_kind: str,
    act_mode: bool,
    source: str,
    replay_labels: Path | None,
    policy_mode: str,
) -> str | None:
    """Refuse a reserved ACT attempt silently turning into fake/teacher/replay."""
    if campaign_mode not in {"teacher", "act"}:
        return "STACK_CAMPAIGN_MODE must be teacher or act"
    if campaign_kind not in {"rollout", "label_replay"}:
        return "STACK_CAMPAIGN_KIND must be rollout or label_replay"
    if campaign_mode == "act":
        if campaign_kind != "rollout":
            return "an ACT-reserved attempt must be a rollout, not label_replay"
        if not act_mode:
            return "ACT-reserved attempt requires --bundle and an ACT runtime config"
        if source != "policy":
            return "ACT-reserved attempt requires action_source: policy"
        if replay_labels is not None:
            return "ACT-reserved attempt cannot use replay labels"
        if policy_mode == "fake":
            return "ACT-reserved attempt cannot select fake policy"
        return None
    if campaign_kind == "label_replay":
        if replay_labels is None:
            return "label_replay requires --replay-labels"
        if act_mode:
            return "label_replay cannot use the ACT provider"
        if source != "policy":
            return "label_replay requires a policy runtime template"
        return None
    if act_mode:
        return "teacher-reserved rollout cannot use the ACT provider"
    if source != "teacher":
        return "teacher-reserved rollout requires action_source: teacher"
    if replay_labels is not None:
        return "teacher-reserved rollout cannot use replay labels"
    return None


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--runtime-config",
        type=Path,
        default=Path(os.environ.get("STACK_RUNTIME_CONFIG", "configs/runtime/native_teacher.yaml")),
    )
    parser.add_argument(
        "--campaign-config", type=Path, default=ROOT / "configs/experiments/stack_campaign.yaml"
    )
    parser.add_argument(
        "--backend-config", type=Path,
        help="explicit backend recipe; its follower must match the runtime recipe",
    )
    parser.add_argument("--seed", type=int, default=os.environ.get("STACK_SEED"))
    parser.add_argument("--run-id", default=os.environ.get("STACK_RUN_ID"))
    parser.add_argument("--episode-id", default=os.environ.get("STACK_EPISODE_ID"))
    parser.add_argument("--output", type=Path, default=os.environ.get("STACK_ATTEMPT_DIR"))
    parser.add_argument("--build-root", type=Path, default=ROOT / ".build/ros-dev")
    parser.add_argument("--policy-sdk", type=Path, default=ROOT / ".build/generated/policy-sdk")
    parser.add_argument(
        "--assets-root", type=Path, default=ROOT / "third_party/src/mujoco_menagerie"
    )
    parser.add_argument("--asset-summary", type=Path, default=ROOT / ".build/assets/panda.json")
    parser.add_argument("--image", default="sim-dual-arm/backend-sim:follower-dev")
    parser.add_argument(
        "--policy-config", type=Path, default=ROOT / "configs/learning/fake_policy.yaml"
    )
    parser.add_argument("--policy-image", default="sim-dual-arm/learning:policy-dev")
    parser.add_argument(
        "--policy-mode", choices=("auto", "fake", "act"), default="auto"
    )
    parser.add_argument("--bundle", type=Path)
    parser.add_argument(
        "--act-policy-config", type=Path, default=ROOT / "configs/learning/act_policy.yaml"
    )
    parser.add_argument("--device", choices=("cpu", "cuda", "cuda:0"))
    parser.add_argument("--profile", type=Path)
    parser.add_argument("--observation-schema", type=Path)
    parser.add_argument("--action-schema", type=Path)
    parser.add_argument("--learning-image", default="sim-dual-arm/learning:act-dev")
    parser.add_argument("--learning-venv", type=Path, default=ROOT / ".build/learning-dev-venv")
    parser.add_argument(
        "--learning-source", type=Path, default=ROOT / "third_party/src/lerobot-act-v0.6.1"
    )
    parser.add_argument("--learning-cache", type=Path, default=ROOT / ".build/cache")
    parser.add_argument("--replay-labels", type=Path)
    parser.add_argument("--timeout", type=float, default=100.0)
    args = parser.parse_args(argv)
    if not os.environ.get("STACK_CAMPAIGN_ATTEMPT_ID"):
        parser.error("a complete task must be reserved by stack_campaign.py before launch")
    if any(value is None for value in (args.seed, args.run_id, args.episode_id, args.output)):
        parser.error("seed/run/episode/output must be supplied by the campaign")
    native_policy.dev_compose.validate_run_id(args.run_id)
    native_policy.dev_compose.validate_run_id(args.episode_id)

    recipe = args.runtime_config.resolve()
    initial = yaml.safe_load(recipe.read_text())
    source = str(initial["action_source"])
    act_mode = args.policy_mode == "act" or (
        args.policy_mode == "auto" and source == "policy" and args.bundle is not None
    )
    campaign_mode = os.environ.get("STACK_CAMPAIGN_MODE", "")
    campaign_kind = os.environ.get("STACK_CAMPAIGN_KIND", "rollout")
    alignment = campaign_alignment_error(
        campaign_mode=campaign_mode,
        campaign_kind=campaign_kind,
        act_mode=act_mode,
        source=source,
        replay_labels=args.replay_labels,
        policy_mode=args.policy_mode,
    )
    if alignment is not None:
        parser.error(alignment)
    settings: ActSettings | None = None
    contract: ActContract | None = None
    if act_mode:
        if args.bundle is None:
            parser.error("--bundle is required for ACT evaluation")
        settings = _act_overrides(load_act_settings(args.act_policy_config.resolve()), args)
        contract = resolve_act_contract(
            bundle=settings.bundle,
            profile_path=settings.profile,
            observation_schema_path=settings.observation_schema,
            action_schema_path=settings.action_schema,
            require_formal=settings.require_formal,
        )
    elif args.policy_mode == "fake" and source == "teacher":
        parser.error("teacher runtime configs cannot use --policy-mode fake")
    elif args.policy_mode == "act" and source == "teacher":
        parser.error("teacher runtime configs cannot use --policy-mode act")

    runtime, backend, act_config, source = generate_inputs(
        recipe,
        args.seed,
        args.run_id,
        settings=settings,
        contract=contract,
        act_template=args.act_policy_config.resolve(),
        backend_template=None if args.backend_config is None else args.backend_config.resolve(),
    )
    del runtime, backend
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=True)
    generated = ROOT / ".build/generated/stack-inputs" / args.run_id
    for path in (generated / "runtime.yaml", generated / "backend.yaml"):
        shutil.copy2(path, output / path.name)
    if act_config is not None:
        shutil.copy2(act_config, output / "act_policy.yaml")
        shutil.copy2(generated / "act_contract.json", output / "act_contract.json")
    if args.replay_labels is not None:
        if os.environ.get("STACK_CAMPAIGN_KIND") != "label_replay":
            parser.error("reference replay requires a reserved label_replay attempt")
        source_path = args.replay_labels.resolve()
        labels = json.loads(source_path.read_text())
        if labels.get("seed") != args.seed:
            parser.error("replay seed must equal the recorded teacher seed")
        runtime_document = yaml.safe_load((output / "runtime.yaml").read_text())
        profile_path = ROOT / runtime_document["robot_profile"].removeprefix("/workspace/")
        profile_document = json.loads(profile_path.read_text())
        profile_hash = hashlib.sha256(
            json.dumps(
                profile_document, sort_keys=True, separators=(",", ":"), allow_nan=False
            ).encode()
        ).hexdigest()
        if labels.get("robot_profile_hash") != profile_hash:
            parser.error("replay profile/calibration differs from the demonstration")
        policy_document = yaml.safe_load(args.policy_config.read_text())
        policy_document["service"]["robot_profile_hash"] = profile_hash
        args.policy_config = generated / "replay-policy.yaml"
        args.policy_config.write_text(yaml.safe_dump(policy_document, sort_keys=False))
        os.environ["STACK_REPLAY_LABELS_HOST"] = str(source_path)
        os.environ["STACK_REPLAY_LABELS"] = "/replay/labels.json"
        write_json(
            output / "replay_source.json",
            {
                "kind": "applied_label_replay",
                "test_only": True,
                "is_act": False,
                "seed": args.seed,
                "source": str(source_path),
                "labels_sha256": hashlib.sha256(source_path.read_bytes()).hexdigest(),
                "robot_profile_hash": profile_hash,
                "source_episode": labels["episode_id"],
                "time_mapping": (
                    "first deployment observation maps to first recorded observation; "
                    "future targets use relative elapsed simulation time"
                ),
                "source_first_observation_sim_time_ns": labels["labels"][0][
                    "observation_sim_time_ns"
                ],
            },
        )
    project = "stack-task-" + uuid.uuid4().hex[:12]
    command = [
        "--config",
        str(generated / "runtime.yaml"),
        "--backend-config",
        str(generated / "backend.yaml"),
        "--stack-mode",
        "teacher" if source == "teacher" else "policy",
        "--policy-mode",
        "act" if act_mode else "fake",
        "--episode-id",
        args.episode_id,
        "--build-root",
        str(args.build_root.resolve()),
        "--policy-sdk",
        str(args.policy_sdk.resolve()),
        "--assets-root",
        str(args.assets_root.resolve()),
        "--asset-summary",
        str(args.asset_summary.resolve()),
        "--policy-config",
        str(args.policy_config.resolve()),
        "--policy-image",
        args.policy_image,
        "--data-root",
        str(output / "stack"),
        "--run-id",
        args.run_id,
        "--project",
        project,
        "--image",
        args.image,
        "--timeout",
        str(args.timeout),
    ]
    if act_mode:
        command.extend(
            [
                "--act-policy-config",
                str((generated / "act_policy.yaml").resolve()),
                "--bundle",
                str(settings.bundle),  # type: ignore[union-attr]
                "--learning-image",
                args.learning_image,
                "--learning-venv",
                str(args.learning_venv.resolve()),
                "--learning-source",
                str(args.learning_source.resolve()),
                "--learning-cache",
                str(args.learning_cache.resolve()),
            ]
        )
    result: dict[str, Any] = {
        "project": project,
        "seed": args.seed,
        "episode_id": args.episode_id,
        "action_source": source,
        "policy_mode": "act" if act_mode else "fake" if source == "policy" else "teacher",
        "attempt_id": os.environ["STACK_CAMPAIGN_ATTEMPT_ID"],
        "campaign_mode": campaign_mode,
        "campaign_kind": campaign_kind,
        "campaign_purpose": os.environ.get("STACK_CAMPAIGN_PURPOSE", ""),
        "inputs": {
            p.name: hashlib.sha256(p.read_bytes()).hexdigest()
            for p in (generated / "runtime.yaml", generated / "backend.yaml")
        },
    }
    if contract is not None:
        result["act"] = contract.summary()
        result["act_config_sha256"] = hashlib.sha256(
            (output / "act_policy.yaml").read_bytes()
        ).hexdigest()
    write_json(output / "launch_result.json", result)
    code = native_policy.main(command)
    result["launcher_exit_code"] = code
    stack_root = output / "stack" / args.run_id
    stack_root.mkdir(parents=True, exist_ok=True)
    shutil.copy2(args.campaign_config.resolve(), stack_root / "campaign.yaml")
    metrics = policy_metrics(stack_root / "act-policy-log")
    if metrics is not None:
        result["act_metrics"] = metrics
        write_json(output / "act_metrics.json", metrics)
    manifests = list((stack_root / "raw" / args.run_id).glob("*/manifest.json"))
    if len(manifests) == 1:
        episode = manifests[0].parent
        for filename in ("manifest.json", "task_result.json"):
            if (episode / filename).exists():
                shutil.copy2(episode / filename, output / filename)
        result["recording_directory"] = str(episode)
        result["export_exit_code"] = export_episode(
            episode,
            stack_root,
            output,
            args.build_root.resolve(),
            args.policy_sdk.resolve(),
            args.image,
            args.run_id,
        )
        export = stack_root / "export"
        if (export / "quality.json").exists():
            shutil.copy2(export / "quality.json", output / "quality.json")
        result["export_directory"] = str(export)
        if result["export_exit_code"] and code == 0:
            code = 1
    else:
        result["error"] = f"expected one manifest, found {len(manifests)}"
        if code == 0:
            code = 1
    result["remaining_owned"] = list(native_backend._project_resources("docker", project))
    if result["remaining_owned"]:
        code = 1
    result["exit_code"] = code
    write_json(output / "launch_result.json", result)
    print(json.dumps(result), flush=True)
    return code


if __name__ == "__main__":
    raise SystemExit(main())
