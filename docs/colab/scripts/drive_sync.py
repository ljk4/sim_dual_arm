"""Local and Colab Google Drive synchronization helpers powered by rclone."""

from __future__ import annotations

import json
import shutil
import subprocess
from dataclasses import dataclass
from pathlib import Path


@dataclass(frozen=True)
class SyncContext:
    """Resolved local and Google Drive paths for a specific robot/backend/task scenario."""

    repo_root: Path
    robot: str
    backend: str
    task: str
    rclone_remote: str
    dataset_archive: str
    local_task_dir: Path
    remote_task_dir: str


def find_repo_root(start: Path | None = None) -> Path:
    """Walk upward from `start` (default cwd) to locate the sim_dual_arm repository root."""
    origin = (start or Path.cwd()).resolve()
    for candidate in (origin, *origin.parents):
        if (candidate / ".git").exists() and (candidate / "shared" / "scenarios").is_dir():
            return candidate
    raise FileNotFoundError(f"未从路径向上找到仓库根目录: {origin}")


def create_context(
    *,
    robot: str,
    backend: str,
    task: str,
    rclone_remote: str = "gdrive:sim_dual_arm",
    dataset_archive: str = "lerobot.tar",
    repo_root: Path | str | None = None,
) -> SyncContext:
    """Validate scenario binding, check local rclone connectivity, and build a SyncContext."""
    root = Path(repo_root).resolve() if repo_root is not None else find_repo_root()
    binding_file = root / "shared" / "scenarios" / robot / backend / task / "binding.yaml"
    if not binding_file.is_file():
        raise FileNotFoundError(f"方案不存在: {binding_file}")

    if shutil.which("rclone") is None:
        raise RuntimeError("未找到 rclone，请先安装并配置好 Google Drive remote。")

    remote_root = rclone_remote.rstrip("/")
    probe = subprocess.run(["rclone", "lsd", remote_root], check=False, capture_output=True, text=True)
    if probe.returncode != 0:
        raise RuntimeError(f"rclone 无法访问 {remote_root}:\n{probe.stderr}")

    ctx = SyncContext(
        repo_root=root,
        robot=robot,
        backend=backend,
        task=task,
        rclone_remote=remote_root,
        dataset_archive=dataset_archive,
        local_task_dir=root / "output" / robot / backend / task,
        remote_task_dir=f"{remote_root}/{robot}/{backend}/{task}",
    )
    print(f"仓库根目录: {ctx.repo_root}")
    print(f"本地任务目录: {ctx.local_task_dir}")
    print(f"云端任务目录: {ctx.remote_task_dir}")
    print("首次在新的 VS Code Colab 运行时注入凭证，可在本机终端复制 base64：")
    print("  base64 -w0 ~/.config/rclone/rclone.conf")
    return ctx


def _run_rclone(args: list[str], *, capture: bool = False) -> subprocess.CompletedProcess[str]:
    cmd = ["rclone", *args]
    if capture:
        return subprocess.run(cmd, check=True, capture_output=True, text=True)
    return subprocess.run(cmd, check=True, text=True)


def _remote_files(remote_dir: str) -> set[str]:
    res = subprocess.run(["rclone", "lsf", remote_dir, "--files-only"], check=False, capture_output=True, text=True)
    if res.returncode != 0:
        return set()
    return {line.strip() for line in res.stdout.splitlines() if line.strip()}


def list_remote_dirs(remote_dir: str) -> list[str]:
    """List immediate subdirectory names in a remote rclone directory."""
    res = subprocess.run(["rclone", "lsf", remote_dir, "--dirs-only"], check=False, capture_output=True, text=True)
    if res.returncode != 0:
        return []
    return sorted(line.strip().rstrip("/") for line in res.stdout.splitlines() if line.strip())


def resolve_model_name(requested: str, candidates: list[str]) -> str:
    """Resolve `'latest'` or an explicit `act_*` model directory name."""
    act_dirs = sorted((name for name in candidates if name.startswith("act_")), reverse=True)
    if requested == "latest":
        if not act_dirs:
            raise FileNotFoundError("未找到任何 act_* 模型目录。")
        return act_dirs[0]
    if requested not in candidates:
        raise FileNotFoundError(f"未找到指定模型目录 {requested!r}，可选: {act_dirs}")
    return requested


def show_status(ctx: SyncContext) -> None:
    """Print read-only local and Google Drive inventory for datasets and ACT models."""
    local_ds = ctx.local_task_dir / "datasets" / "lerobot"
    local_manifest = local_ds / "manifest.json"
    print("=== 本地数据集 ===")
    if local_manifest.is_file():
        m = json.loads(local_manifest.read_text(encoding="utf-8"))
        splits = m.get("splits", {})
        tr, va = splits.get("training", {}), splits.get("validation", {})
        print(
            f"  {local_ds} | status={m.get('status')} | "
            f"train={tr.get('episode_count')}({tr.get('frame_count')}f) | "
            f"val={va.get('episode_count')}({va.get('frame_count')}f)"
        )
    else:
        print(f"  (无) {local_ds}")

    print("\n=== 云端数据集 ===")
    res = subprocess.run(["rclone", "lsl", f"{ctx.remote_task_dir}/datasets"], check=False, capture_output=True, text=True)
    print(res.stdout.strip() or "  (空或不存在)")

    print("\n=== 本地模型 ===")
    local_models_dir = ctx.local_task_dir / "models"
    local_models = (
        sorted((p for p in local_models_dir.iterdir() if p.is_dir()), key=lambda p: p.name, reverse=True)
        if local_models_dir.is_dir()
        else []
    )
    if local_models:
        for p in local_models:
            sf = p / "model.safetensors"
            size_mb = sf.stat().st_size / (1024 * 1024) if sf.is_file() else 0.0
            print(f"  - {p.name} (model.safetensors={size_mb:.2f} MiB)")
    else:
        print("  (无)")

    print("\n=== 云端模型 ===")
    remote_models = sorted((d for d in list_remote_dirs(f"{ctx.remote_task_dir}/models") if d.startswith("act_")), reverse=True)
    if remote_models:
        for name in remote_models:
            print(f"  - {name}")
    else:
        print("  (无)")


def upload_dataset(ctx: SyncContext) -> None:
    """Pack local `datasets/lerobot` into a tar archive and upload it to Google Drive."""
    local_datasets = ctx.local_task_dir / "datasets"
    local_lerobot = local_datasets / "lerobot"
    if not (local_lerobot / "manifest.json").is_file():
        raise FileNotFoundError(f"未找到转换后的数据集: {local_lerobot / 'manifest.json'}")

    tmp_tar = Path("/tmp") / f"{ctx.robot}_{ctx.backend}_{ctx.task}_{ctx.dataset_archive}"
    remote_archive = f"{ctx.remote_task_dir}/datasets/{ctx.dataset_archive}"

    try:
        print(f"正在打包数据集: {local_lerobot} -> {tmp_tar}")
        subprocess.run(["tar", "-C", str(local_datasets), "-cf", str(tmp_tar), "lerobot"], check=True)
        size_mb = tmp_tar.stat().st_size / (1024 * 1024)
        print(f"打包完成 ({size_mb:.2f} MiB)，开始上传至 {remote_archive}...")
        _run_rclone(["mkdir", f"{ctx.remote_task_dir}/datasets"])
        _run_rclone(["copyto", str(tmp_tar), remote_archive, "--progress"])
    finally:
        if tmp_tar.exists():
            tmp_tar.unlink()

    print("上传完成，云端文件信息:")
    print(_run_rclone(["lsl", remote_archive], capture=True).stdout.strip())


def download_dataset(ctx: SyncContext, *, overwrite_local: bool = False) -> Path:
    """Download dataset tar (or unpacked directory) from Google Drive to `datasets/lerobot`."""
    local_datasets = ctx.local_task_dir / "datasets"
    target_dir = local_datasets / "lerobot"
    if target_dir.exists() and not overwrite_local:
        raise FileExistsError(f"本地数据集已存在: {target_dir}；若需覆盖请设 overwrite_local=True")

    local_datasets.mkdir(parents=True, exist_ok=True)
    staging_dir = local_datasets / ".unpack_tmp"
    if staging_dir.exists():
        shutil.rmtree(staging_dir)
    staging_dir.mkdir(parents=True, exist_ok=True)
    tmp_tar = Path("/tmp") / f"{ctx.robot}_{ctx.backend}_{ctx.task}_{ctx.dataset_archive}"
    remote_datasets = f"{ctx.remote_task_dir}/datasets"

    try:
        if ctx.dataset_archive in _remote_files(remote_datasets):
            remote_archive = f"{remote_datasets}/{ctx.dataset_archive}"
            print(f"正在通过 rclone 下载压缩包: {remote_archive} -> {tmp_tar}")
            _run_rclone(["copyto", remote_archive, str(tmp_tar), "--progress"])
            print(f"正在解压到本地盘: {staging_dir}")
            subprocess.run(["tar", "-xf", str(tmp_tar), "-C", str(staging_dir)], check=True)
            if (staging_dir / "lerobot" / "manifest.json").is_file():
                extracted_root = staging_dir / "lerobot"
            elif (staging_dir / "manifest.json").is_file():
                extracted_root = staging_dir
            else:
                raise FileNotFoundError(
                    f"{remote_archive} 解压后未找到 manifest.json（应在根级或 lerobot/ 下）"
                )
        elif "manifest.json" in _remote_files(f"{remote_datasets}/lerobot"):
            extracted_root = staging_dir / "lerobot"
            print(f"未找到 {ctx.dataset_archive}，改用远端已解压目录: {remote_datasets}/lerobot")
            _run_rclone(["copy", f"{remote_datasets}/lerobot", str(extracted_root), "--transfers", "8", "--progress"])
        else:
            raise FileNotFoundError(
                f"在 {remote_datasets} 未找到 {ctx.dataset_archive} 或 lerobot/manifest.json，"
                "请先在本机运行 docs/colab/sync_drive_rclone.ipynb 上传数据集"
            )

        required = ("manifest.json", "train", "validation", "processors")
        missing = [name for name in required if not (extracted_root / name).exists()]
        if missing:
            raise RuntimeError(f"下载的数据集缺少必要项: {missing} ({extracted_root})")

        if target_dir.exists():
            shutil.rmtree(target_dir)
        shutil.move(str(extracted_root), str(target_dir))
    finally:
        if tmp_tar.exists():
            tmp_tar.unlink()
        if staging_dir.exists():
            shutil.rmtree(staging_dir, ignore_errors=True)

    manifest = json.loads((target_dir / "manifest.json").read_text(encoding="utf-8"))
    splits = manifest.get("splits", {})
    tr = splits.get("training", {})
    va = splits.get("validation", {})
    print(f"本地数据集就绪: {target_dir}")
    print(
        f"status={manifest.get('status')} | synthetic={manifest.get('synthetic')} | "
        f"train={tr.get('episode_count')} 条/{tr.get('frame_count')} 帧 | "
        f"validation={va.get('episode_count')} 条/{va.get('frame_count')} 帧"
    )
    return target_dir


def download_model(
    ctx: SyncContext,
    *,
    model_name: str = "latest",
    sync_training_logs: bool = True,
    overwrite_local: bool = False,
) -> Path:
    """Download an ACT model directory (and optionally training logs) from Google Drive."""
    remote_models = list_remote_dirs(f"{ctx.remote_task_dir}/models")
    chosen = resolve_model_name(model_name, remote_models)

    local_model = ctx.local_task_dir / "models" / chosen
    if local_model.exists() and not overwrite_local:
        raise FileExistsError(f"本地模型已存在: {local_model}；若需覆盖请设 overwrite_local=True")

    local_model.parent.mkdir(parents=True, exist_ok=True)
    staging_model = local_model.parent / f".tmp_{chosen}"
    if staging_model.exists():
        shutil.rmtree(staging_model)

    try:
        remote_model = f"{ctx.remote_task_dir}/models/{chosen}"
        print(f"正在下载模型: {remote_model} -> {local_model}")
        _run_rclone(["copy", remote_model, str(staging_model), "--transfers", "4", "--progress"])
        if not (staging_model / "model.safetensors").is_file():
            raise FileNotFoundError(f"下载结果缺少 model.safetensors: {staging_model}")
        if local_model.exists():
            shutil.rmtree(local_model)
        shutil.move(str(staging_model), str(local_model))
    finally:
        if staging_model.exists():
            shutil.rmtree(staging_model, ignore_errors=True)

    if sync_training_logs and chosen in list_remote_dirs(f"{ctx.remote_task_dir}/logs/training"):
        local_log = ctx.local_task_dir / "logs" / "training" / chosen
        local_log.mkdir(parents=True, exist_ok=True)
        _run_rclone(["copy", f"{ctx.remote_task_dir}/logs/training/{chosen}", str(local_log), "--transfers", "4", "--progress"])
        print(f"训练日志已同步: {local_log}")

    print(f"模型下载完成: {local_model}")
    for item in sorted(local_model.iterdir()):
        if item.is_file():
            print(f"  - {item.name}: {item.stat().st_size / (1024 * 1024):.2f} MiB")
    args_file = local_model / "training_args.yaml"
    if args_file.is_file():
        print("\n--- training_args.yaml ---")
        print(args_file.read_text(encoding="utf-8").strip())
    return local_model


def upload_model(
    ctx: SyncContext,
    *,
    model_name: str = "latest",
    sync_training_logs: bool = True,
) -> None:
    """Upload a local ACT model directory (and optionally training logs) to Google Drive."""
    local_models_dir = ctx.local_task_dir / "models"
    local_dirs = sorted(p.name for p in local_models_dir.iterdir() if p.is_dir()) if local_models_dir.is_dir() else []
    chosen = resolve_model_name(model_name, local_dirs)
    local_model = local_models_dir / chosen
    if not (local_model / "model.safetensors").is_file():
        raise FileNotFoundError(f"本地模型目录缺少 model.safetensors: {local_model}")

    remote_model = f"{ctx.remote_task_dir}/models/{chosen}"
    _run_rclone(["mkdir", f"{ctx.remote_task_dir}/models"])
    print(f"正在上传模型: {local_model} -> {remote_model}")
    _run_rclone(["copy", str(local_model), remote_model, "--transfers", "4", "--progress"])

    local_log = ctx.local_task_dir / "logs" / "training" / chosen
    if sync_training_logs and local_log.is_dir():
        remote_log = f"{ctx.remote_task_dir}/logs/training/{chosen}"
        _run_rclone(["mkdir", f"{ctx.remote_task_dir}/logs/training"])
        _run_rclone(["copy", str(local_log), remote_log, "--transfers", "4", "--progress"])
        print(f"训练日志已上传: {remote_log}")

    print(f"模型上传完成: {remote_model}")
