#!/usr/bin/env python3
"""Probe GPU VRAM ceiling and DataLoader pipeline throughput for ACT training."""

from __future__ import annotations

import argparse
import gc
import json
import os
import shutil
import statistics
import sys
import time
from pathlib import Path
from typing import Any


def _default_project_root() -> Path:
    env_root = os.environ.get("PROJECT_ROOT")
    if env_root:
        return Path(env_root).resolve()
    return Path(__file__).resolve().parents[3]


def _parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--project-root",
        type=Path,
        default=_default_project_root(),
        help="repository root (default: PROJECT_ROOT or inferred from script path)",
    )
    parser.add_argument("--config", type=Path, default=None, help="direct ACT training YAML path")
    parser.add_argument(
        "--robot", help="robot id under <shared-root>/scenarios/<robot>/<backend>/<task>/"
    )
    parser.add_argument(
        "--backend", help="backend id under <shared-root>/scenarios/<robot>/<backend>/<task>/"
    )
    parser.add_argument("--task", help="task id for --robot/--backend")
    parser.add_argument("--shared-root", type=Path, default=None, help="shared scenarios root")
    parser.add_argument(
        "--mode",
        choices=("smoke", "overfit", "full"),
        default="full",
        help="training mode to benchmark (default: full)",
    )
    parser.add_argument(
        "--max-batch-size",
        type=int,
        default=256,
        help="upper bound for probed batch_size (clamped to TrainingConfig limit <= 256)",
    )
    parser.add_argument(
        "--vram-target",
        type=float,
        default=0.85,
        help="maximum safe fraction of total GPU VRAM to allocate (default: 0.85)",
    )
    parser.add_argument(
        "--ram-target",
        type=float,
        default=0.70,
        help="maximum safe fraction of available RAM / /dev/shm to budget (default: 0.70)",
    )
    parser.add_argument("--warmup-steps", type=int, default=2, help="warmup steps per trial")
    parser.add_argument("--measure-steps", type=int, default=6, help="timed steps per trial")
    parser.add_argument("--output", type=Path, default=None, help="optional JSON output path")
    args = parser.parse_args(argv)

    if args.robot is None and args.backend is None and args.task is None:
        if args.config is None:
            parser.error("either --robot/--backend/--task or --config is required")
    elif args.robot is None or args.backend is None or args.task is None:
        parser.error("--robot, --backend, and --task must be passed together")
    elif args.config is not None:
        parser.error("--config cannot be combined with --robot/--backend/--task")
    if args.shared_root is not None and args.robot is None:
        parser.error("--shared-root requires --robot/--backend/--task")
    if not 1 <= args.max_batch_size <= 256:
        parser.error("--max-batch-size must be in 1..256")
    if not 0.1 <= args.vram_target <= 0.98:
        parser.error("--vram-target must be in 0.1..0.98")
    if not 0.1 <= args.ram_target <= 0.95:
        parser.error("--ram-target must be in 0.1..0.95")
    if args.warmup_steps < 0 or args.measure_steps < 1:
        parser.error("--warmup-steps >= 0 and --measure-steps >= 1 are required")
    return args


def _read_meminfo() -> tuple[int, int]:
    """Return (total_ram_bytes, available_ram_bytes) from /proc/meminfo."""
    values: dict[str, int] = {}
    with Path("/proc/meminfo").open(encoding="utf-8") as fh:
        for line in fh:
            if ":" not in line:
                continue
            key, rest = line.split(":", 1)
            if key in {"MemTotal", "MemAvailable"}:
                parts = rest.strip().split()
                if parts:
                    values[key] = int(parts[0]) * 1024
    total = values.get("MemTotal", 0)
    avail = values.get("MemAvailable", total)
    return total, avail


def _process_tree_rss_bytes(root_pid: int) -> int:
    """Sum VmRSS across root_pid and all its living descendant processes."""
    children: dict[int, list[int]] = {}
    rss_by_pid: dict[int, int] = {}
    proc_root = Path("/proc")
    for entry in proc_root.iterdir():
        if not entry.name.isdigit():
            continue
        pid = int(entry.name)
        try:
            text = (entry / "status").read_text(encoding="utf-8")
        except (OSError, UnicodeDecodeError):
            continue
        ppid: int | None = None
        rss_bytes = 0
        for line in text.splitlines():
            if line.startswith("PPid:"):
                try:
                    ppid = int(line.split(":", 1)[1].strip())
                except ValueError:
                    ppid = None
            elif line.startswith("VmRSS:"):
                parts = line.split(":", 1)[1].strip().split()
                if parts:
                    try:
                        rss_bytes = int(parts[0]) * 1024
                    except ValueError:
                        rss_bytes = 0
        if ppid is not None:
            children.setdefault(ppid, []).append(pid)
        rss_by_pid[pid] = rss_bytes

    total_rss = 0
    pending = [root_pid]
    seen: set[int] = set()
    while pending:
        pid = pending.pop()
        if pid in seen:
            continue
        seen.add(pid)
        total_rss += rss_by_pid.get(pid, 0)
        pending.extend(children.get(pid, ()))
    return total_rss


def _is_oom(error: BaseException) -> bool:
    import torch

    if isinstance(error, torch.cuda.OutOfMemoryError):
        return True
    if isinstance(error, RuntimeError):
        return "out of memory" in str(error).lower()
    return False


def _clear_cuda() -> None:
    import torch

    gc.collect()
    if torch.cuda.is_available():
        torch.cuda.synchronize()
        torch.cuda.empty_cache()


def _resolve_config(args: argparse.Namespace) -> tuple[Any, Any | None]:
    from dual_arm_learning.config import load_training_config
    from dual_arm_learning.scenario import (
        artifact_paths,
        check_dataset_recipe_consistency,
        check_profile_consistency,
        select_scenario,
    )
    from robot_contracts.project_paths import project_path

    root = args.project_root.resolve()
    selection = None
    if args.robot is not None:
        selection = select_scenario(
            robot=args.robot,
            backend=args.backend,
            task=args.task,
            recipe_field="training",
            stage="learning",
            project_root=root,
            shared_root=args.shared_root,
        )
        config_path = selection.recipe_path
    else:
        config_path = project_path(args.config, root)

    config = load_training_config(config_path, root)
    updates: dict[str, Any] = {"mode": args.mode}
    if selection is not None:
        paths = artifact_paths(selection.robot, selection.backend, selection.task, root)
        updates["data_root"] = paths["dataset"]
        updates["cache_root"] = paths["cache"]
        updates["checkpoint"] = config.checkpoint.model_copy(update={"root": paths["models"]})
        updates["logging"] = config.logging.model_copy(update={"root": paths["training_logs"]})
        check_profile_consistency(selection, config.dataset.profile_path)
        check_dataset_recipe_consistency(selection, config.dataset_config_path)
    return config.model_copy(update=updates), selection


def _batch_tensor_bytes(value: Any) -> int:
    import torch

    if isinstance(value, torch.Tensor):
        return value.numel() * value.element_size()
    if isinstance(value, dict):
        return sum(_batch_tensor_bytes(v) for v in value.values())
    if isinstance(value, (list, tuple)):
        return sum(_batch_tensor_bytes(v) for v in value)
    return 0


def _slice_batch(batch: dict[str, Any], count: int) -> dict[str, Any]:
    import torch

    sliced: dict[str, Any] = {}
    for key, value in batch.items():
        if isinstance(value, torch.Tensor) or isinstance(value, list):
            sliced[key] = value[:count]
        else:
            sliced[key] = value
    return sliced


def _run_one_step(
    *,
    policy: Any,
    pre: Any,
    optimizer: Any,
    scaler: Any,
    amp_torch_dtype: Any,
    grad_clip_norm: float,
    keys: tuple[str, ...],
    batch: dict[str, Any],
    process_batch_fn: Any,
) -> bool:
    """Run one forward/backward/step attempt. Return True if optimizer stepped, False if GradScaler skipped."""
    import torch

    processed = process_batch_fn(pre, batch, keys)
    optimizer.zero_grad(set_to_none=True)
    with torch.amp.autocast(device_type="cuda", dtype=amp_torch_dtype, enabled=True):
        loss, _ = policy.forward(processed)
    if not torch.isfinite(loss):
        raise RuntimeError("non-finite training loss during hardware probe")
    scaler_enabled = bool(scaler.is_enabled())
    scaler.scale(loss).backward()
    scaler.unscale_(optimizer)
    grad_norm = torch.nn.utils.clip_grad_norm_(policy.parameters(), grad_clip_norm)
    scale_before = scaler.get_scale() if scaler_enabled else 0.0
    scaler.step(optimizer)
    scaler.update()
    if scaler_enabled and scaler.get_scale() < scale_before:
        return False
    if not torch.isfinite(grad_norm):
        raise RuntimeError("non-finite gradient norm during hardware probe")
    return True


def _probe_vram_stage(
    *,
    config: Any,
    amp_dtype: str,
    total_vram_bytes: int,
    vram_target: float,
    max_batch_size: int,
    eligible_indices: tuple[int, ...] | list[int],
    train_dataset: Any,
    validation_dataset: Any | None,
    policy: Any,
    pre: Any,
    optimizer: Any,
) -> tuple[list[dict[str, Any]], int, int, int]:
    import torch
    from torch.utils.data import default_collate

    from dual_arm_learning.training.support import process_batch, sample_keys

    amp_torch_dtype = torch.bfloat16 if amp_dtype == "bfloat16" else torch.float16
    keys = sample_keys(config)
    total_vram_gb = total_vram_bytes / (1024**3)

    cap_bs = min(256, max_batch_size, len(eligible_indices))
    raw_candidates = [config.batch_size, 64, 96, 128, 160, 192, 224, 256]
    candidates = sorted({bs for bs in raw_candidates if 1 <= bs <= cap_bs})
    if not candidates:
        candidates = [max(1, cap_bs)]

    max_bs = candidates[-1]
    print(f"[Stage 1/2] 解码 {max_bs} 帧真实样本并探测 GPU 显存上限 (candidates={candidates})...", flush=True)
    samples = [train_dataset[eligible_indices[i]] for i in range(max_bs)]
    base_batch = default_collate(samples)
    sample_bytes = max(1, _batch_tensor_bytes(base_batch) // max_bs)
    print(f"            单样本张量大小: {sample_bytes / (1024 * 1024):.2f} MiB", flush=True)

    results: list[dict[str, Any]] = []
    for bs in candidates:
        _clear_cuda()
        torch.cuda.reset_peak_memory_stats()
        scaler = torch.amp.GradScaler("cuda", enabled=(amp_dtype == "float16"))
        sub_batch = _slice_batch(base_batch, bs)
        try:
            policy.train(True)
            step_times: list[float] = []
            for _ in range(2):
                retries = 0
                while True:
                    torch.cuda.synchronize()
                    t0 = time.monotonic()
                    stepped = _run_one_step(
                        policy=policy,
                        pre=pre,
                        optimizer=optimizer,
                        scaler=scaler,
                        amp_torch_dtype=amp_torch_dtype,
                        grad_clip_norm=config.grad_clip_norm,
                        keys=keys,
                        batch=sub_batch,
                        process_batch_fn=process_batch,
                    )
                    torch.cuda.synchronize()
                    dt = time.monotonic() - t0
                    if stepped:
                        step_times.append(dt)
                        break
                    retries += 1
                    if retries >= 16:
                        raise RuntimeError("GradScaler overflowed repeatedly during VRAM probe")

            if validation_dataset is not None:
                val_bs = min(bs, len(validation_dataset))
                val_batch = _slice_batch(sub_batch, val_bs)
                was_training = bool(policy.training)
                policy.eval()
                try:
                    with torch.no_grad():
                        processed_val = process_batch(pre, val_batch, keys)
                        model_batch = {
                            k: v for k, v in processed_val.items() if k.startswith("observation.")
                        }
                        _ = policy.predict_action_chunk(model_batch)
                    torch.cuda.synchronize()
                finally:
                    policy.train(was_training)

            peak_alloc_bytes = torch.cuda.max_memory_allocated()
            peak_res_bytes = torch.cuda.max_memory_reserved()
            peak_vram_gb = peak_alloc_bytes / (1024**3)
            peak_res_gb = peak_res_bytes / (1024**3)
            vram_pct = (peak_alloc_bytes / total_vram_bytes) * 100.0
            gpu_step_s = step_times[-1]
            gpu_samples_per_s = bs / max(1e-6, gpu_step_s)
            safe = (peak_alloc_bytes <= total_vram_bytes * vram_target) and (
                peak_res_bytes <= total_vram_bytes * 0.95
            )
            entry = {
                "batch_size": bs,
                "oom": False,
                "safe": bool(safe),
                "peak_vram_gb": round(peak_vram_gb, 3),
                "peak_reserved_gb": round(peak_res_gb, 3),
                "vram_pct": round(vram_pct, 1),
                "gpu_step_s": round(gpu_step_s, 4),
                "gpu_samples_per_s": round(gpu_samples_per_s, 2),
            }
            results.append(entry)
            status = "SAFE" if safe else "OVER_TARGET"
            print(
                f"  bs={bs:3d} | {status:11s} | peak_alloc={peak_vram_gb:5.2f} GiB ({vram_pct:4.1f}%) | "
                f"reserved={peak_res_gb:5.2f} GiB | gpu={gpu_samples_per_s:6.1f} samples/s",
                flush=True,
            )
            if not safe and peak_res_bytes > total_vram_bytes * 0.95:
                break
        except Exception as err:
            if not _is_oom(err):
                raise
            results.append(
                {
                    "batch_size": bs,
                    "oom": True,
                    "safe": False,
                    "peak_vram_gb": round(total_vram_gb, 3),
                    "peak_reserved_gb": round(total_vram_gb, 3),
                    "vram_pct": 100.0,
                    "gpu_step_s": None,
                    "gpu_samples_per_s": 0.0,
                }
            )
            print(f"  bs={bs:3d} | OOM         | 停止继续增大 batch_size", flush=True)
            optimizer.zero_grad(set_to_none=True)
            _clear_cuda()
            break

    del samples, base_batch
    _clear_cuda()

    safe_entries = [r for r in results if r["safe"]]
    if safe_entries:
        bs_max = max(safe_entries, key=lambda r: r["batch_size"])["batch_size"]
        bs_fast = max(safe_entries, key=lambda r: r["gpu_samples_per_s"])["batch_size"]
    else:
        non_oom = [r for r in results if not r["oom"]]
        if not non_oom:
            raise RuntimeError("all probed batch sizes hit CUDA OOM")
        smallest = min(non_oom, key=lambda r: r["batch_size"])
        bs_max = smallest["batch_size"]
        bs_fast = smallest["batch_size"]

    return results, bs_max, bs_fast, sample_bytes


def _run_pipeline_trial(
    *,
    config: Any,
    amp_dtype: str,
    bs: int,
    nw: int,
    pf: int,
    warmup_steps: int,
    measure_steps: int,
    total_vram_bytes: int,
    total_ram_bytes: int,
    train_dataset: Any,
    eligible_indices: tuple[int, ...] | None,
    policy: Any,
    pre: Any,
    optimizer: Any,
) -> dict[str, Any]:
    import torch

    from dual_arm_learning.training.support import loader, process_batch, sample_keys

    amp_torch_dtype = torch.bfloat16 if amp_dtype == "bfloat16" else torch.float16
    keys = sample_keys(config)
    trial_cfg = config.model_copy(
        update={
            "batch_size": bs,
            "num_workers": nw,
            "prefetch_factor": pf,
            "pin_memory": True,
            "use_amp": True,
            "amp_dtype": amp_dtype,
        }
    )

    _clear_cuda()
    torch.cuda.reset_peak_memory_stats()
    scaler = torch.amp.GradScaler("cuda", enabled=(amp_dtype == "float16"))
    dl = loader(train_dataset, trial_cfg, training=True, indices=eligible_indices)
    it = iter(dl)
    rss_bytes = 0
    try:
        policy.train(True)

        def _next_batch() -> dict[str, Any]:
            nonlocal it
            try:
                return next(it)
            except StopIteration:
                it = iter(dl)
                return next(it)

        for _ in range(warmup_steps):
            retries = 0
            while True:
                batch = _next_batch()
                stepped = _run_one_step(
                    policy=policy,
                    pre=pre,
                    optimizer=optimizer,
                    scaler=scaler,
                    amp_torch_dtype=amp_torch_dtype,
                    grad_clip_norm=config.grad_clip_norm,
                    keys=keys,
                    batch=batch,
                    process_batch_fn=process_batch,
                )
                if stepped:
                    break
                retries += 1
                if retries >= 16:
                    raise RuntimeError("GradScaler overflowed repeatedly during warmup")
        torch.cuda.synchronize()

        data_wait_total = 0.0
        compute_total = 0.0
        step_wall_times: list[float] = []
        for _ in range(measure_steps):
            step_data_wait = 0.0
            step_compute = 0.0
            retries = 0
            while True:
                t_d0 = time.monotonic()
                batch = _next_batch()
                t_d1 = time.monotonic()
                step_data_wait += t_d1 - t_d0

                torch.cuda.synchronize()
                t_c0 = time.monotonic()
                stepped = _run_one_step(
                    policy=policy,
                    pre=pre,
                    optimizer=optimizer,
                    scaler=scaler,
                    amp_torch_dtype=amp_torch_dtype,
                    grad_clip_norm=config.grad_clip_norm,
                    keys=keys,
                    batch=batch,
                    process_batch_fn=process_batch,
                )
                torch.cuda.synchronize()
                t_c1 = time.monotonic()
                step_compute += t_c1 - t_c0
                if stepped:
                    break
                retries += 1
                if retries >= 16:
                    raise RuntimeError("GradScaler overflowed repeatedly during measure")
            data_wait_total += step_data_wait
            compute_total += step_compute
            step_wall_times.append(step_data_wait + step_compute)

        rss_bytes = _process_tree_rss_bytes(os.getpid())
        peak_alloc_bytes = torch.cuda.max_memory_allocated()
    finally:
        it = None
        dl = None
        optimizer.zero_grad(set_to_none=True)
        _clear_cuda()

    total_s = max(1e-6, data_wait_total + compute_total)
    steps_per_sec = measure_steps / total_s
    samples_per_sec = (measure_steps * bs) / total_s
    data_wait_pct = (data_wait_total / total_s) * 100.0
    median_step_s = statistics.median(step_wall_times)
    peak_vram_gb = peak_alloc_bytes / (1024**3)
    vram_pct = (peak_alloc_bytes / total_vram_bytes) * 100.0
    rss_gb = rss_bytes / (1024**3)
    ram_pct = (rss_bytes / total_ram_bytes) * 100.0 if total_ram_bytes > 0 else 0.0

    return {
        "batch_size": bs,
        "num_workers": nw,
        "prefetch_factor": pf,
        "samples_per_sec": round(samples_per_sec, 2),
        "steps_per_sec": round(steps_per_sec, 3),
        "median_step_s": round(median_step_s, 4),
        "data_wait_s": round(data_wait_total / measure_steps, 4),
        "compute_s": round(compute_total / measure_steps, 4),
        "data_wait_pct": round(data_wait_pct, 1),
        "peak_vram_gb": round(peak_vram_gb, 3),
        "vram_pct": round(vram_pct, 1),
        "process_tree_rss_gb": round(rss_gb, 3),
        "ram_pct": round(ram_pct, 1),
    }


def _probe_pipeline_stage(
    *,
    config: Any,
    amp_dtype: str,
    bs_max: int,
    bs_fast: int,
    sample_bytes: int,
    cpu_count: int,
    total_vram_bytes: int,
    total_ram_bytes: int,
    avail_ram_bytes: int,
    shm_free_bytes: int,
    ram_target: float,
    warmup_steps: int,
    measure_steps: int,
    train_dataset: Any,
    eligible_indices: tuple[int, ...] | None,
    policy: Any,
    pre: Any,
    optimizer: Any,
) -> list[dict[str, Any]]:
    loader_multiplier = 2 if config.mode_spec.validation else 1

    def within_budget(bs: int, nw: int, pf: int) -> bool:
        est_queue = loader_multiplier * nw * pf * bs * sample_bytes
        est_worker_overhead = loader_multiplier * nw * int(0.35 * (1024**3))
        shm_ok = est_queue <= shm_free_bytes * ram_target
        ram_ok = (est_queue + est_worker_overhead) <= avail_ram_bytes * ram_target
        return shm_ok and ram_ok

    raw_nw = {2, 4, max(2, cpu_count // 2), max(2, cpu_count - 1), cpu_count}
    worker_candidates = sorted({nw for nw in raw_nw if 1 <= nw <= min(32, cpu_count)})
    if not worker_candidates:
        worker_candidates = [1]

    print(
        f"\n[Stage 2/2] 端到端 DataLoader + GPU 流水线扫参 "
        f"(bs_max={bs_max}, bs_fast={bs_fast}, cpus={cpu_count}, loader_multiplier={loader_multiplier})...",
        flush=True,
    )

    trials: list[dict[str, Any]] = []
    seen: set[tuple[int, int, int]] = set()

    def run_trial(bs: int, nw: int, pf: int) -> dict[str, Any]:
        key = (bs, nw, pf)
        for existing in trials:
            if (existing["batch_size"], existing["num_workers"], existing["prefetch_factor"]) == key:
                return existing
        seen.add(key)
        res = _run_pipeline_trial(
            config=config,
            amp_dtype=amp_dtype,
            bs=bs,
            nw=nw,
            pf=pf,
            warmup_steps=warmup_steps,
            measure_steps=measure_steps,
            total_vram_bytes=total_vram_bytes,
            total_ram_bytes=total_ram_bytes,
            train_dataset=train_dataset,
            eligible_indices=eligible_indices,
            policy=policy,
            pre=pre,
            optimizer=optimizer,
        )
        trials.append(res)
        print(
            f"  bs={bs:3d} nw={nw:2d} pf={pf:2d} | {res['samples_per_sec']:6.1f} samples/s "
            f"({res['steps_per_sec']:5.2f} step/s) | wait={res['data_wait_pct']:4.1f}% | "
            f"VRAM={res['peak_vram_gb']:4.2f}G ({res['vram_pct']:4.1f}%) | "
            f"RSS={res['process_tree_rss_gb']:4.2f}G ({res['ram_pct']:4.1f}%)",
            flush=True,
        )
        return res

    # Pass 1: sweep num_workers at (bs_max, nw, 2)
    nw_trials: list[dict[str, Any]] = []
    for nw in worker_candidates:
        if within_budget(bs_max, nw, 2):
            nw_trials.append(run_trial(bs_max, nw, 2))
    if not nw_trials:
        fallback_nw = min(2, cpu_count)
        print(
            f"  [注意] /dev/shm 或可用内存较紧，回退测试保守配置 (bs={bs_max}, nw={fallback_nw}, pf=2)",
            flush=True,
        )
        nw_trials.append(run_trial(bs_max, fallback_nw, 2))

    nw_best = max(nw_trials, key=lambda r: r["samples_per_sec"])["num_workers"]

    # Pass 2: sweep prefetch_factor at (bs_max, nw_best, pf)
    pf_trials: list[dict[str, Any]] = [
        r for r in trials if r["batch_size"] == bs_max and r["num_workers"] == nw_best
    ]
    for pf in (2, 4, 6, 8):
        if (bs_max, nw_best, pf) in seen:
            continue
        if within_budget(bs_max, nw_best, pf):
            pf_trials.append(run_trial(bs_max, nw_best, pf))
    pf_best = max(pf_trials, key=lambda r: r["samples_per_sec"])["prefetch_factor"]

    # Pass 3: check bs_fast if different from bs_max
    if bs_fast != bs_max:
        if within_budget(bs_fast, nw_best, pf_best):
            run_trial(bs_fast, nw_best, pf_best)
        elif within_budget(bs_fast, nw_best, 2):
            run_trial(bs_fast, nw_best, 2)

    return trials


def _print_summary(
    *,
    hardware: dict[str, Any],
    baseline: dict[str, Any],
    recommended: dict[str, Any],
    best_trial: dict[str, Any],
    trials: list[dict[str, Any]],
) -> None:
    print("\n=== 端到端吞吐测试汇总 ===", flush=True)
    header = (
        f"{'batch':>5} {'nw':>3} {'pf':>3} | {'samples/s':>9} {'step/s':>7} "
        f"{'wait%':>6} | {'VRAM(GiB)':>11} {'RSS(GiB)':>11}"
    )
    print(header, flush=True)
    print("-" * len(header), flush=True)
    for row in sorted(trials, key=lambda r: r["samples_per_sec"], reverse=True):
        vram_col = f"{row['peak_vram_gb']:4.2f} ({row['vram_pct']:4.1f}%)"
        rss_col = f"{row['process_tree_rss_gb']:4.2f} ({row['ram_pct']:4.1f}%)"
        print(
            f"{row['batch_size']:5d} {row['num_workers']:3d} {row['prefetch_factor']:3d} | "
            f"{row['samples_per_sec']:9.1f} {row['steps_per_sec']:7.2f} "
            f"{row['data_wait_pct']:5.1f}% | {vram_col:>11} {rss_col:>11}",
            flush=True,
        )

    wait_pct = best_trial["data_wait_pct"]
    if wait_pct >= 30.0:
        bottleneck = f"数据解码/读取偏慢 (data_wait={wait_pct:.1f}%)"
    elif wait_pct <= 10.0:
        bottleneck = f"GPU 算力饱和良好 (data_wait={wait_pct:.1f}%)"
    else:
        bottleneck = f"CPU 数据喂送与 GPU 计算较均衡 (data_wait={wait_pct:.1f}%)"

    step_ratio = recommended["batch_size"] / max(1, baseline["batch_size"])
    print("\n=== 推荐最优训练参数 ===", flush=True)
    print(
        f"GPU: {hardware['gpu_name']} ({hardware['vram_gb']} GiB, CC {hardware['compute_capability']}) | "
        f"CPU: {hardware['cpu_count']} 核 | RAM: {hardware['ram_total_gb']} GiB | /dev/shm: {hardware['shm_free_gb']} GiB",
        flush=True,
    )
    print(
        f"推荐参数: batch_size={recommended['batch_size']} (默认 {baseline['batch_size']}) | "
        f"num_workers={recommended['num_workers']} | prefetch_factor={recommended['prefetch_factor']} | "
        f"amp_dtype={recommended['amp_dtype']}",
        flush=True,
    )
    print(f"瓶颈诊断: {bottleneck}", flush=True)
    if step_ratio > 1.0:
        print(
            f"样本等效提示: batch_size 扩大了 {step_ratio:.2f}x，每步消化更多样本；"
            f"同等 epoch/样本量仅需原步数的 1/{step_ratio:.2f}。",
            flush=True,
        )


def main(argv: list[str] | None = None) -> int:
    args = _parse_args(argv)
    import torch

    if not torch.cuda.is_available():
        print("error: probe_colab_training.py requires a CUDA GPU", file=sys.stderr)
        return 1

    from dual_arm_learning.training.manifest import sample_filter, verify_dataset_manifest
    from dual_arm_learning.training.support import (
        build_optimizer_param_groups,
        build_policy,
        build_processors,
        load_dataset,
    )

    config, selection = _resolve_config(args)
    cap = torch.cuda.get_device_capability(0)
    amp_dtype = "bfloat16" if cap[0] >= 8 else "float16"
    config = config.model_copy(update={"device": "cuda", "use_amp": True, "amp_dtype": amp_dtype})

    total_vram_bytes = int(torch.cuda.get_device_properties(0).total_memory)
    total_ram_bytes, avail_ram_bytes = _read_meminfo()
    shm_usage = shutil.disk_usage("/dev/shm")
    cpu_count = os.cpu_count() or 2

    hardware = {
        "gpu_name": torch.cuda.get_device_name(0),
        "compute_capability": f"{cap[0]}.{cap[1]}",
        "native_bf16": bool(cap[0] >= 8),
        "vram_gb": round(total_vram_bytes / (1024**3), 2),
        "cpu_count": cpu_count,
        "ram_total_gb": round(total_ram_bytes / (1024**3), 2),
        "ram_available_gb": round(avail_ram_bytes / (1024**3), 2),
        "shm_total_gb": round(shm_usage.total / (1024**3), 2),
        "shm_free_gb": round(shm_usage.free / (1024**3), 2),
    }

    print("=== 服务器硬件与训练吞吐探测 ===", flush=True)
    if selection is not None:
        print(
            f"方案: {selection.robot}/{selection.backend}/{selection.task} | "
            f"Binding: {selection.resolved.path}",
            flush=True,
        )
    print(f"配方: {config.source} | 模式: {config.mode}", flush=True)
    print(
        f"硬件: {hardware['gpu_name']} ({hardware['vram_gb']} GiB, CC {hardware['compute_capability']}, "
        f"amp={amp_dtype}) | CPU={cpu_count} 核 | RAM={hardware['ram_available_gb']}/{hardware['ram_total_gb']} GiB | "
        f"/dev/shm={hardware['shm_free_gb']}/{hardware['shm_total_gb']} GiB",
        flush=True,
    )
    print(
        f"预算: max_batch_size={args.max_batch_size} | vram_target={args.vram_target:.0%} | "
        f"ram_target={args.ram_target:.0%} | warmup={args.warmup_steps} | measure={args.measure_steps}",
        flush=True,
    )

    audit = verify_dataset_manifest(config, require_full=(config.mode == "full"))
    mask = sample_filter(config, audit)

    train_episodes = None if config.mode_spec.episodes is None else tuple(config.mode_spec.episodes)
    train_dataset = load_dataset(config, audit.train_root, train_episodes, training=True)
    validation_dataset = (
        load_dataset(
            config,
            audit.validation_root,
            tuple(range(len(audit.validation_episodes))),
            training=False,
        )
        if config.mode_spec.validation
        else None
    )

    eligible_indices: tuple[int, ...] | None = None if mask is None else mask.eligible_indices
    probe_indices: tuple[int, ...] = (
        eligible_indices if eligible_indices is not None else tuple(range(len(train_dataset)))
    )

    policy, act_config, _, _ = build_policy(config)
    pre, _ = build_processors(config, train_dataset, act_config)
    optimizer = torch.optim.AdamW(
        build_optimizer_param_groups(policy, config),
        lr=config.lr,
        weight_decay=config.weight_decay,
    )

    vram_sweep, bs_max, bs_fast, sample_bytes = _probe_vram_stage(
        config=config,
        amp_dtype=amp_dtype,
        total_vram_bytes=total_vram_bytes,
        vram_target=args.vram_target,
        max_batch_size=args.max_batch_size,
        eligible_indices=probe_indices,
        train_dataset=train_dataset,
        validation_dataset=validation_dataset,
        policy=policy,
        pre=pre,
        optimizer=optimizer,
    )

    pipeline_sweep = _probe_pipeline_stage(
        config=config,
        amp_dtype=amp_dtype,
        bs_max=bs_max,
        bs_fast=bs_fast,
        sample_bytes=sample_bytes,
        cpu_count=cpu_count,
        total_vram_bytes=total_vram_bytes,
        total_ram_bytes=total_ram_bytes,
        avail_ram_bytes=avail_ram_bytes,
        shm_free_bytes=shm_usage.free,
        ram_target=args.ram_target,
        warmup_steps=args.warmup_steps,
        measure_steps=args.measure_steps,
        train_dataset=train_dataset,
        eligible_indices=eligible_indices,
        policy=policy,
        pre=pre,
        optimizer=optimizer,
    )

    best_trial = max(pipeline_sweep, key=lambda r: r["samples_per_sec"])
    baseline = {
        "batch_size": config.batch_size,
        "num_workers": config.num_workers,
        "prefetch_factor": config.prefetch_factor,
        "amp_dtype": config.amp_dtype,
    }
    recommended = {
        "batch_size": int(best_trial["batch_size"]),
        "num_workers": int(best_trial["num_workers"]),
        "prefetch_factor": int(best_trial["prefetch_factor"]),
        "amp_dtype": amp_dtype,
    }
    report = {
        "hardware": hardware,
        "baseline": baseline,
        "recommended": recommended,
        "metrics": best_trial,
        "vram_sweep": vram_sweep,
        "pipeline_sweep": pipeline_sweep,
    }

    _print_summary(
        hardware=hardware,
        baseline=baseline,
        recommended=recommended,
        best_trial=best_trial,
        trials=pipeline_sweep,
    )

    if args.output is not None:
        out_path = args.output.resolve()
        out_path.parent.mkdir(parents=True, exist_ok=True)
        out_path.write_text(json.dumps(report, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
        print(f"\n探测报告已写入: {out_path}", flush=True)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
