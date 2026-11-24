"""Colab training orchestration: repo sync, setup, probe, batch-step scaling, loss plots, and backup."""

from __future__ import annotations

import json
import math
import os
import shlex
import subprocess
import threading
from pathlib import Path
from typing import Any

from .drive_sync import SyncContext, download_dataset
from .loss_plot import display_loss_plot, load_metrics, render_loss_plot


def compute_scaled_steps(
    base_steps: int,
    target_batch_size: int,
    *,
    reference_batch_size: int = 32,
    step_multiple: int = 1000,
    auto_scale: bool = True,
) -> int:
    """Scale training steps inversely with batch size and round UP to a multiple of `step_multiple`.

    When `auto_scale` is False, returns `int(base_steps)` unchanged.
    """
    if base_steps <= 0:
        raise ValueError(f"base_steps 必须大于 0: {base_steps}")
    if target_batch_size <= 0:
        raise ValueError(f"target_batch_size 必须大于 0: {target_batch_size}")
    if reference_batch_size <= 0:
        raise ValueError(f"reference_batch_size 必须大于 0: {reference_batch_size}")
    if step_multiple <= 0:
        raise ValueError(f"step_multiple 必须大于 0: {step_multiple}")

    if not auto_scale:
        return int(base_steps)

    numerator = int(base_steps) * int(reference_batch_size)
    denominator = int(target_batch_size) * int(step_multiple)
    units = (numerator + denominator - 1) // denominator
    return max(int(step_multiple), units * int(step_multiple))


def _stream_cmd(cmd: list[str], *, cwd: Path | None = None, shell_prefix: str | None = None) -> None:
    if shell_prefix:
        cmd_str = f"{shell_prefix} && exec {shlex.join(cmd)}"
        popen_args = ["bash", "-lc", cmd_str]
    else:
        popen_args = cmd
    proc = subprocess.Popen(
        popen_args,
        cwd=str(cwd) if cwd is not None else None,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        bufsize=1,
    )
    assert proc.stdout is not None
    for line in proc.stdout:
        print(line, end="", flush=True)
    ret = proc.wait()
    if ret != 0:
        raise subprocess.CalledProcessError(ret, cmd)


def sync_repo(*, work_dir: Path | str, repo_url: str, repo_branch: str) -> Path:
    """Clone or fast-forward update the public repository into `work_dir`."""
    work = Path(work_dir)
    if (work / ".git").is_dir():
        print(f"仓库已存在，更新分支 {repo_branch}...")
        subprocess.run(["git", "-C", str(work), "fetch", "origin", repo_branch], check=True)
        subprocess.run(["git", "-C", str(work), "checkout", repo_branch], check=True)
        subprocess.run(["git", "-C", str(work), "pull", "--ff-only", "origin", repo_branch], check=True)
    else:
        print(f"克隆公开仓库 {repo_url} ({repo_branch}) 到 {work}...")
        subprocess.run(
            ["git", "clone", "--branch", repo_branch, "--single-branch", repo_url, str(work)],
            check=True,
        )
    commit = subprocess.run(
        ["git", "-C", str(work), "log", "-1", "--oneline"],
        check=True,
        capture_output=True,
        text=True,
    ).stdout.strip()
    print(f"当前提交: {commit}")
    return work


def setup_learning_env(
    *,
    work_dir: Path | str,
    rclone_remote: str,
    use_drive_cache: bool = True,
    torch_index_url: str = "",
) -> None:
    """Bootstrap `.venv`, `third_party`, Policy SDK, and `.env.learning`, syncing `_cache` via rclone."""
    work = Path(work_dir)
    remote_cache = f"{rclone_remote.rstrip('/')}/_cache"
    cache_local = Path("/tmp/sda_rclone_cache")

    script_path = work / "docs" / "colab" / "scripts" / "setup_colab_learning.sh"
    args = ["bash", str(script_path), "--repo-root", str(work)]

    if use_drive_cache:
        cache_local.mkdir(parents=True, exist_ok=True)
        print(f"从 {remote_cache} 拉取缓存...")
        subprocess.run(["rclone", "copy", remote_cache, str(cache_local), "--transfers", "4"], check=False)
        args.extend(["--cache-dir", str(cache_local)])

    if torch_index_url.strip():
        args.extend(["--torch-index-url", torch_index_url.strip()])

    _stream_cmd(args, cwd=work)

    if use_drive_cache:
        print(f"回传新增缓存到 {remote_cache}...")
        subprocess.run(["rclone", "copy", str(cache_local), remote_cache, "--transfers", "4"], check=False)


def fetch_dataset(
    *,
    work_dir: Path | str,
    rclone_remote: str,
    robot: str,
    task: str,
    dataset_archive: str = "lerobot.tar",
) -> Path:
    """Fetch and unpack the LeRobot dataset into `var/<robot>/<task>/datasets/lerobot`."""
    work = Path(work_dir).resolve()
    remote_root = rclone_remote.rstrip("/")
    ctx = SyncContext(
        repo_root=work,
        robot=robot,
        task=task,
        rclone_remote=remote_root,
        dataset_archive=dataset_archive,
        local_task_dir=work / "var" / robot / task,
        remote_task_dir=f"{remote_root}/{robot}/{task}",
    )
    return download_dataset(ctx, overwrite_local=True)


def probe_hardware(
    *,
    work_dir: Path | str,
    robot: str,
    task: str,
    mode: str = "full",
    auto_probe: bool = True,
    max_batch_size: int = 256,
    vram_target_ratio: float = 0.85,
    ram_target_ratio: float = 0.70,
) -> dict[str, Any] | None:
    """Run `probe_colab_training.py` and write `colab_probe_result.json`."""
    if not auto_probe:
        print("AUTO_PROBE=False，跳过硬件吞吐探测。")
        return None

    work = Path(work_dir).resolve()
    probe_script = work / "docs" / "colab" / "scripts" / "probe_colab_training.py"
    probe_output = work / "var" / robot / task / "debug" / "colab_probe_result.json"
    probe_args = [
        "python3",
        str(probe_script),
        "--robot",
        robot,
        "--task",
        task,
        "--mode",
        mode,
        "--max-batch-size",
        str(int(max_batch_size)),
        "--vram-target",
        str(float(vram_target_ratio)),
        "--ram-target",
        str(float(ram_target_ratio)),
        "--output",
        str(probe_output),
    ]
    _stream_cmd(
        probe_args,
        shell_prefix=f"cd {shlex.quote(str(work))} && source .env.learning",
    )
    report = json.loads(probe_output.read_text(encoding="utf-8"))
    print("已保存探测推荐参数:", json.dumps(report["recommended"], ensure_ascii=False))
    return report


def _fmt_hms(seconds: float | None) -> str:
    if seconds is None:
        return "--:--:--"
    total = max(0, round(seconds))
    hours, rem = divmod(total, 3600)
    minutes, secs = divmod(rem, 60)
    return f"{hours:02d}:{minutes:02d}:{secs:02d}"


def _read_base_training_config(venv_python: Path, src_cfg: Path, mode: str) -> dict[str, Any]:
    snippet = (
        "import json, sys, yaml; "
        "doc = yaml.safe_load(open(sys.argv[1], encoding='utf-8')); "
        "tr = doc['training']; "
        "va = tr.get('visual_augmentation') or {}; "
        "print(json.dumps({"
        "'amp_dtype': tr.get('amp_dtype', 'bfloat16'), "
        "'batch_size': int(tr['batch_size']), "
        "'num_workers': int(tr.get('num_workers', 4)), "
        "'prefetch_factor': int(tr.get('prefetch_factor', 2)), "
        "'lr': float(tr['lr']), "
        "'lr_backbone': None if tr.get('lr_backbone') is None else float(tr['lr_backbone']), "
        "'visual_aug_enabled': bool(va.get('enabled', False)), "
        "'camera_dropout_prob': float(va.get('camera_dropout_prob', 0.10)), "
        "'default_steps': int(doc['modes'][sys.argv[2]]['steps'])"
        "}))"
    )
    proc = subprocess.run(
        [str(venv_python), "-c", snippet, str(src_cfg), mode],
        check=True,
        capture_output=True,
        text=True,
    )
    return json.loads(proc.stdout.strip())


def _write_temp_training_config(venv_python: Path, src_cfg: Path, dst_cfg: Path, mode: str, overrides: dict[str, Any]) -> None:
    snippet = (
        "import json, sys, yaml; "
        "src, dst, mode, raw = sys.argv[1:5]; "
        "ov = json.loads(raw); "
        "doc = yaml.safe_load(open(src, encoding='utf-8')); "
        "doc['training']['amp_dtype'] = ov['amp_dtype']; "
        "doc['training']['batch_size'] = ov['batch_size']; "
        "doc['training']['num_workers'] = ov['num_workers']; "
        "doc['training']['prefetch_factor'] = ov['prefetch_factor']; "
        "doc['training']['lr'] = ov['lr']; "
        "doc['training']['lr_backbone'] = ov['lr_backbone']; "
        "va = dict(doc['training'].get('visual_augmentation') or {}); "
        "va['enabled'] = ov['visual_aug_enabled']; "
        "va['camera_dropout_prob'] = ov['camera_dropout_prob']; "
        "doc['training']['visual_augmentation'] = va; "
        "(doc['modes'][mode].update({'steps': ov['steps']}) if ov['steps'] is not None else None); "
        "yaml.safe_dump(doc, open(dst, 'w', encoding='utf-8'), sort_keys=False, allow_unicode=True)"
    )
    subprocess.run(
        [str(venv_python), "-c", snippet, str(src_cfg), str(dst_cfg), mode, json.dumps(overrides)],
        check=True,
    )


def run_training(
    *,
    work_dir: Path | str,
    robot: str,
    task: str,
    mode: str = "full",
    base_steps: int | None = 20000,
    reference_batch_size: int = 32,
    auto_scale_steps: bool = True,
    step_multiple: int = 1000,
    plot_every_steps: int = 1000,
    max_steps: int | None = None,
    auto_probe: bool = True,
    batch_size: int | None = None,
    num_workers: int | None = None,
    prefetch_factor: int | None = None,
    scale_lr_sqrt: bool = False,
    visual_augment: bool | None = None,
    camera_dropout_prob: float | None = None,
) -> Path | None:
    """Launch ACT training with batch-based step scaling and periodic loss-curve rendering."""
    work = Path(work_dir).resolve()
    venv_python = work / ".venv" / "bin" / "python"
    src_cfg = work / "shared" / "scenarios" / robot / task / "configs" / "learning" / "act_training.yaml"
    probe_output = work / "var" / robot / task / "debug" / "colab_probe_result.json"

    base_info = _read_base_training_config(venv_python, src_cfg, mode)
    base_amp_dtype = base_info["amp_dtype"]
    base_batch_size = int(base_info["batch_size"])
    base_num_workers = int(base_info["num_workers"])
    base_prefetch_factor = int(base_info["prefetch_factor"])
    base_lr = float(base_info["lr"])
    base_lr_backbone = base_info["lr_backbone"]
    base_aug_enabled = bool(base_info["visual_aug_enabled"])
    base_cam_dropout = float(base_info["camera_dropout_prob"])
    default_steps = int(base_info["default_steps"])

    if auto_probe and probe_output.is_file():
        rec = json.loads(probe_output.read_text(encoding="utf-8"))["recommended"]
        target_amp_dtype = str(rec["amp_dtype"])
        target_batch_size = int(rec["batch_size"])
        target_num_workers = int(rec["num_workers"])
        target_prefetch_factor = int(rec["prefetch_factor"])
        param_source = f"硬件探测 ({probe_output.name})"
    else:
        gpu_probe = json.loads(
            subprocess.run(
                [
                    str(venv_python),
                    "-c",
                    (
                        "import json, torch; "
                        "cuda_ok = torch.cuda.is_available(); "
                        "cap = list(torch.cuda.get_device_capability(0)) if cuda_ok else [0, 0]; "
                        "print(json.dumps({'amp_dtype': 'bfloat16' if (cuda_ok and cap[0] >= 8) else 'float16'}))"
                    ),
                ],
                check=True,
                capture_output=True,
                text=True,
            ).stdout.strip()
        )
        target_amp_dtype = str(gpu_probe["amp_dtype"])
        target_batch_size = base_batch_size
        target_num_workers = min(8, max(2, os.cpu_count() or 2))
        target_prefetch_factor = base_prefetch_factor
        param_source = "默认配方 + 硬件能力检测"

    if batch_size is not None:
        target_batch_size = int(batch_size)
    if num_workers is not None:
        target_num_workers = int(num_workers)
    if prefetch_factor is not None:
        target_prefetch_factor = int(prefetch_factor)

    target_lr = base_lr
    target_lr_backbone = base_lr_backbone
    if scale_lr_sqrt and target_batch_size != base_batch_size:
        lr_scale = math.sqrt(target_batch_size / base_batch_size)
        target_lr = base_lr * lr_scale
        if base_lr_backbone is not None:
            target_lr_backbone = base_lr_backbone * lr_scale

    target_aug_enabled = base_aug_enabled if visual_augment is None else bool(visual_augment)
    target_cam_dropout = base_cam_dropout if camera_dropout_prob is None else float(camera_dropout_prob)

    if max_steps is not None:
        target_steps = int(max_steps)
        step_desc = f"手动 MAX_STEPS={target_steps}"
    elif mode == "full":
        effective_base_steps = int(base_steps) if base_steps is not None else default_steps
        target_steps = compute_scaled_steps(
            effective_base_steps,
            target_batch_size,
            reference_batch_size=reference_batch_size,
            step_multiple=step_multiple,
            auto_scale=auto_scale_steps,
        )
        if auto_scale_steps:
            step_desc = (
                f"基准 {effective_base_steps} @ batch={reference_batch_size} -> "
                f"batch={target_batch_size} 向上取整至 {step_multiple} 的倍数 = {target_steps}"
            )
        else:
            step_desc = f"固定基准步数 {target_steps}（已禁用按 batch 缩放）"
    else:
        target_steps = default_steps
        step_desc = f"{mode} 模式默认步数 {target_steps}"

    steps_differ_from_default = target_steps != default_steps
    needs_temp_cfg = (
        target_amp_dtype != base_amp_dtype
        or target_batch_size != base_batch_size
        or target_num_workers != base_num_workers
        or target_prefetch_factor != base_prefetch_factor
        or target_lr != base_lr
        or target_lr_backbone != base_lr_backbone
        or target_aug_enabled != base_aug_enabled
        or target_cam_dropout != base_cam_dropout
        or steps_differ_from_default
    )

    print(f"参数来源: {param_source}")
    print(f"步数策略: {step_desc}")
    print(
        f"有效训练参数: batch_size={target_batch_size} | num_workers={target_num_workers} | "
        f"prefetch_factor={target_prefetch_factor} | amp_dtype={target_amp_dtype} | "
        f"lr={target_lr:g} | lr_backbone={target_lr_backbone} | steps={target_steps} | "
        f"visual_aug={target_aug_enabled} | camera_dropout_prob={target_cam_dropout:g}"
    )

    train_args = ["bash", "learning/lerobot/scripts/train.sh"]
    if needs_temp_cfg:
        debug_dir = work / "var" / robot / task / "debug"
        debug_dir.mkdir(parents=True, exist_ok=True)
        temp_cfg = debug_dir / "act_training_colab.yaml"
        overrides = {
            "amp_dtype": target_amp_dtype,
            "batch_size": target_batch_size,
            "num_workers": target_num_workers,
            "prefetch_factor": target_prefetch_factor,
            "lr": target_lr,
            "lr_backbone": target_lr_backbone,
            "visual_aug_enabled": target_aug_enabled,
            "camera_dropout_prob": target_cam_dropout,
            "steps": target_steps if steps_differ_from_default else None,
        }
        _write_temp_training_config(venv_python, src_cfg, temp_cfg, mode, overrides)
        print(f"已写入临时训练配置: {temp_cfg}")
        train_args.extend(["--config", str(temp_cfg)])
    else:
        print("使用方案默认训练配方。")
        train_args.extend(["--robot", robot, "--task", task])

    train_args.extend(["--mode", mode, "--max-steps", str(target_steps)])

    logs_root = work / "var" / robot / task / "logs" / "training"
    existing_runs = (
        {p.name for p in logs_root.iterdir() if p.is_dir() and p.name.startswith("act_")}
        if logs_root.is_dir()
        else set()
    )

    stop_event = threading.Event()
    io_lock = threading.Lock()
    state: dict[str, Any] = {"metrics_path": None, "plot_path": None, "display_id": None}
    plot_title = f"{robot}/{task} ({mode}, batch={target_batch_size}, steps={target_steps})"

    def _find_active_metrics() -> Path | None:
        if state["metrics_path"] is not None:
            return state["metrics_path"]
        if not logs_root.is_dir():
            return None
        new_runs = sorted(
            (p for p in logs_root.iterdir() if p.is_dir() and p.name.startswith("act_") and p.name not in existing_runs),
            key=lambda p: p.name,
        )
        if not new_runs:
            return None
        candidate = new_runs[-1] / "metrics.jsonl"
        if candidate.is_file():
            state["metrics_path"] = candidate
            return candidate
        return None

    def _refresh_plot(metrics_path: Path) -> Path | None:
        out_path = metrics_path.parent / "loss_curve.png"
        rendered = render_loss_plot(metrics_path, out_path, title=plot_title)
        if rendered is not None:
            state["plot_path"] = rendered
            with io_lock:
                state["display_id"] = display_loss_plot(rendered, display_id=state["display_id"])
        return rendered

    def _watch_progress() -> None:
        last_train_step = 0
        last_val_step = -1
        last_plot_bucket = 0
        best_val: float | None = None

        while not stop_event.wait(15.0):
            metrics_path = _find_active_metrics()
            if metrics_path is None:
                continue
            train_rows, val_rows = load_metrics(metrics_path)
            if not train_rows and not val_rows:
                continue

            should_plot = False
            if val_rows:
                val_vals = [float(r["val_l1"]) for r in val_rows if r.get("val_l1") is not None]
                if val_vals:
                    best_val = min(val_vals)
                latest_val = val_rows[-1]
                val_step = int(latest_val["step"])
                if val_step > last_val_step:
                    last_val_step = val_step
                    if plot_every_steps > 0 and val_step >= plot_every_steps:
                        should_plot = True
                    with io_lock:
                        best_str = f"{best_val:.5f}" if best_val is not None else "--"
                        print(
                            f"[验证] step={val_step} | val_l1={float(latest_val['val_l1']):.5f} | best_val={best_str}",
                            flush=True,
                        )

            if train_rows:
                latest_train = train_rows[-1]
                step = int(latest_train["step"])
                if plot_every_steps > 0:
                    bucket = step // plot_every_steps
                    if bucket > last_plot_bucket:
                        last_plot_bucket = bucket
                        should_plot = True
                if step > last_train_step:
                    last_train_step = step
                    elapsed = float(latest_train.get("elapsed_seconds", 0.0))
                    pct = min(100.0, step / max(1, target_steps) * 100.0)
                    eta = (elapsed * max(0, target_steps - step) / step) if step > 0 else None
                    best_part = f" | best_val={best_val:.5f}" if best_val is not None else ""
                    with io_lock:
                        print(
                            f"[进度] {step}/{target_steps} ({pct:5.1f}%) | "
                            f"loss={float(latest_train['loss']):.5f} | "
                            f"l1={float(latest_train.get('l1_loss', 0.0)):.5f} | "
                            f"grad={float(latest_train.get('grad_norm', 0.0)):.2f}{best_part} | "
                            f"用时 {_fmt_hms(elapsed)} | 预计剩余 {_fmt_hms(eta)}",
                            flush=True,
                        )

            if should_plot:
                _refresh_plot(metrics_path)

    cmd_str = f"cd {shlex.quote(str(work))} && source .env.learning && exec {shlex.join(train_args)}"
    with io_lock:
        print("执行命令:", shlex.join(train_args), flush=True)

    watcher = threading.Thread(target=_watch_progress, name="colab-train-progress", daemon=True)
    watcher.start()
    proc = subprocess.Popen(
        ["bash", "-lc", cmd_str],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        bufsize=1,
    )
    try:
        assert proc.stdout is not None
        for line in proc.stdout:
            with io_lock:
                print(line, end="", flush=True)
        ret = proc.wait()
    finally:
        stop_event.set()
        watcher.join(timeout=2.0)

    final_metrics = _find_active_metrics()
    if final_metrics is not None:
        final_plot = _refresh_plot(final_metrics)
        if final_plot is not None:
            print(f"[完成] 已保存训练/验证损失图: {final_plot}")

    if ret != 0:
        raise subprocess.CalledProcessError(ret, train_args)

    return state["plot_path"]


def sync_results_to_drive(
    *,
    work_dir: Path | str,
    rclone_remote: str,
    robot: str,
    task: str,
) -> Path:
    """Upload the newest trained ACT model, training logs (including loss_curve.png), and probe files."""
    local_var = Path(work_dir) / "var" / robot / task
    models_dir = local_var / "models"
    candidates = sorted(
        (p for p in models_dir.iterdir() if p.is_dir() and p.name.startswith("act_")),
        key=lambda p: p.name,
        reverse=True,
    )
    if not candidates:
        raise FileNotFoundError(f"未找到训练产出的模型目录: {models_dir}")

    latest_model = candidates[0]
    if not (latest_model / "model.safetensors").is_file():
        raise FileNotFoundError(f"模型目录缺少 model.safetensors: {latest_model}")

    remote_task_root = f"{rclone_remote.rstrip('/')}/{robot}/{task}"
    remote_model = f"{remote_task_root}/models/{latest_model.name}"
    subprocess.run(
        ["rclone", "copy", str(latest_model), remote_model, "--transfers", "4", "--progress"],
        check=True,
    )

    src_log = local_var / "logs" / "training" / latest_model.name
    remote_log = f"{remote_task_root}/logs/training/{latest_model.name}"
    if src_log.is_dir():
        metrics_file = src_log / "metrics.jsonl"
        loss_png = src_log / "loss_curve.png"
        if metrics_file.is_file() and not loss_png.is_file():
            render_loss_plot(metrics_file, loss_png, title=f"{robot}/{task} ({latest_model.name})")
        subprocess.run(
            ["rclone", "copy", str(src_log), remote_log, "--transfers", "4", "--progress"],
            check=True,
        )

    debug_dir = local_var / "debug"
    for extra_name in ("colab_probe_result.json", "act_training_colab.yaml"):
        extra_file = debug_dir / extra_name
        if extra_file.is_file():
            subprocess.run(
                ["rclone", "copyto", str(extra_file), f"{remote_log}/{extra_name}"],
                check=True,
            )

    print(f"模型已同步至 Google Drive: {remote_model}")
    for item in sorted(latest_model.iterdir()):
        if item.is_file():
            size_mb = item.stat().st_size / (1024 * 1024)
            print(f"  - {item.name}: {size_mb:.2f} MiB")
    print(f"训练日志与探测记录已同步至 Google Drive: {remote_log}")
    print("\n在本机运行 docs/colab/sync_drive_rclone.ipynb 第 5 节即可一键下载模型至:")
    print(f"  var/{robot}/{task}/models/{latest_model.name}/")
    return latest_model
