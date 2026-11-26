"""Non-blocking rclone authentication and Google Drive remote setup for Colab."""

from __future__ import annotations

import base64
import binascii
import os
import re
import shutil
import subprocess
from pathlib import Path


def _remote_name_from_spec(rclone_remote: str) -> str:
    remote_name = rclone_remote.split(":", 1)[0].strip()
    if not remote_name:
        raise ValueError(f"无效的 rclone remote 规范: {rclone_remote!r}")
    return remote_name


def decode_rclone_conf(raw_text: str, remote_name: str) -> str:
    """Decode raw INI or base64-encoded rclone.conf text and validate the target section."""
    cleaned = raw_text.strip()
    if not cleaned:
        raise ValueError("rclone 配置内容为空")

    section_pattern = re.compile(rf"(?m)^\[{re.escape(remote_name)}\]\s*$")
    if section_pattern.search(cleaned):
        decoded = cleaned
    else:
        compact = re.sub(r"\s+", "", cleaned)
        try:
            decoded_bytes = base64.b64decode(compact, validate=True)
        except (binascii.Error, ValueError) as exc:
            # Try standard base64 with missing padding tolerated only when safe, else error clearly
            padded = compact + ("=" * ((4 - len(compact) % 4) % 4))
            try:
                decoded_bytes = base64.b64decode(padded, validate=True)
            except (binascii.Error, ValueError):
                raise ValueError(
                    f"既不是包含 [{remote_name}] 的 INI 配置，也不是有效的 base64 编码字符串"
                ) from exc
        try:
            decoded = decoded_bytes.decode("utf-8").strip()
        except UnicodeDecodeError as exc:
            raise ValueError("base64 解码结果不是有效的 UTF-8 文本") from exc

    if not section_pattern.search(decoded):
        raise ValueError(f"配置中未找到 [{remote_name}] 段")

    return decoded + "\n"


def _write_rclone_conf(conf_text: str, conf_path: Path | None = None) -> Path:
    target = conf_path or (Path.home() / ".config" / "rclone" / "rclone.conf")
    target.parent.mkdir(parents=True, exist_ok=True)
    target.write_text(conf_text, encoding="utf-8")
    os.chmod(target, 0o600)
    return target


def ensure_rclone_installed() -> str:
    """Ensure `rclone` binary is available on PATH and return its executable path."""
    existing = shutil.which("rclone")
    if existing:
        return existing

    print("[信息] 未检测到 rclone，正在安装...")
    install_cmd = "curl -fsSL https://rclone.org/install.sh | bash"
    if os.geteuid() != 0 and shutil.which("sudo"):
        install_cmd = "curl -fsSL https://rclone.org/install.sh | sudo bash"
    subprocess.run(["bash", "-lc", install_cmd], check=True)

    installed = shutil.which("rclone")
    if not installed:
        raise RuntimeError("rclone 安装完成后仍未在 PATH 中找到可执行文件")
    return installed


def verify_and_prepare_remote(
    rclone_remote: str, robot: str, backend: str, task: str
) -> None:
    """Verify connectivity to `rclone_remote` and ensure task and cache directories exist."""
    rclone_bin = ensure_rclone_installed()
    subprocess.run([rclone_bin, "lsd", rclone_remote], check=True)
    remote_root = rclone_remote.rstrip("/")
    for subdir in (f"{remote_root}/_cache", f"{remote_root}/{robot}/{backend}/{task}"):
        subprocess.run([rclone_bin, "mkdir", subdir], check=True)
    print(f"[完成] rclone 远端连通性验证通过，已就绪目录: {remote_root}/{robot}/{backend}/{task}")


def _print_gpu_status() -> None:
    nvidia_smi = shutil.which("nvidia-smi")
    if not nvidia_smi:
        print("[提示] 当前为 CPU 运行时（可用于大内存数据转换；执行训练前请切换至 GPU 运行时）。")
        return
    proc = subprocess.run([nvidia_smi, "--query-gpu=name,memory.total,driver_version", "--format=csv,noheader"], capture_output=True, text=True)
    if proc.returncode == 0 and proc.stdout.strip():
        print(f"[GPU] {proc.stdout.strip()}")
    else:
        subprocess.run([nvidia_smi], check=False)


def _print_memory_status() -> None:
    cpu_count = os.cpu_count() or 1
    mem_total_bytes = 0
    mem_avail_bytes = 0
    try:
        for line in Path("/proc/meminfo").read_text(encoding="utf-8").splitlines():
            if line.startswith("MemTotal:"):
                mem_total_bytes = int(line.split(":", 1)[1].strip().split()[0]) * 1024
            elif line.startswith("MemAvailable:"):
                mem_avail_bytes = int(line.split(":", 1)[1].strip().split()[0]) * 1024
    except (OSError, ValueError, IndexError):
        pass
    shm_usage = shutil.disk_usage("/dev/shm") if Path("/dev/shm").exists() else None
    shm_part = (
        f" | /dev/shm: {shm_usage.free / (1024**3):.1f}/{shm_usage.total / (1024**3):.1f} GiB"
        if shm_usage is not None
        else ""
    )
    print(
        f"[资源] CPU: {cpu_count} 核 | RAM: {mem_avail_bytes / (1024**3):.1f}/"
        f"{mem_total_bytes / (1024**3):.1f} GiB{shm_part}"
    )


def _try_load_existing_conf(remote_name: str, use_colab_secrets: bool) -> Path | None:
    conf_path = Path.home() / ".config" / "rclone" / "rclone.conf"
    if conf_path.is_file():
        try:
            decode_rclone_conf(conf_path.read_text(encoding="utf-8"), remote_name)
            return conf_path
        except ValueError:
            pass

    content_conf = Path("/content/rclone.conf")
    if content_conf.is_file():
        decoded = decode_rclone_conf(content_conf.read_text(encoding="utf-8"), remote_name)
        written = _write_rclone_conf(decoded, conf_path)
        content_conf.unlink(missing_ok=True)
        print("[信息] 已从 /content/rclone.conf 导入配置并删除临时文件。")
        return written

    for env_key in ("RCLONE_CONF_B64", "RCLONE_CONF"):
        env_val = os.environ.get(env_key, "").strip()
        if env_val:
            decoded = decode_rclone_conf(env_val, remote_name)
            written = _write_rclone_conf(decoded, conf_path)
            print(f"[信息] 已从环境变量 {env_key} 导入 rclone 配置。")
            return written

    if use_colab_secrets:
        try:
            from google.colab import userdata  # type: ignore[import-not-found]

            for secret_key in ("RCLONE_CONF_B64", "RCLONE_CONF"):
                try:
                    secret_val = (userdata.get(secret_key) or "").strip()
                except Exception:
                    secret_val = ""
                if secret_val:
                    decoded = decode_rclone_conf(secret_val, remote_name)
                    written = _write_rclone_conf(decoded, conf_path)
                    print(f"[信息] 已从 Colab Secrets ({secret_key}) 导入 rclone 配置。")
                    return written
        except ImportError:
            pass

    return None


def _prompt_with_ipywidgets(
    *, remote_name: str, rclone_remote: str, robot: str, backend: str, task: str
) -> None:
    try:
        import ipywidgets as widgets  # type: ignore[import-not-found]
        from IPython.display import display  # type: ignore[import-not-found]
    except ImportError as exc:
        raise RuntimeError(
            "未找到 ~/.config/rclone/rclone.conf，且当前环境不支持 ipywidgets；"
            "请上传 /content/rclone.conf 或设置环境变量 RCLONE_CONF_B64。"
        ) from exc

    password_box = widgets.Password(
        placeholder="粘贴 `base64 -w0 ~/.config/rclone/rclone.conf` 输出或原始配置文本",
        description="rclone配置:",
        layout=widgets.Layout(width="520px"),
    )
    submit_btn = widgets.Button(description="保存并验证连通性", button_style="primary")
    status_out = widgets.Output()

    def _on_submit(_btn: object) -> None:
        with status_out:
            status_out.clear_output()
            try:
                decoded = decode_rclone_conf(password_box.value, remote_name)
                _write_rclone_conf(decoded)
                password_box.value = ""
                verify_and_prepare_remote(rclone_remote, robot, backend, task)
            except Exception as exc:  # noqa: BLE001
                print(f"[错误] {exc}")

    submit_btn.on_click(_on_submit)
    print(
        f"[等待输入] 未检测到有效的 [{remote_name}] rclone 配置。\n"
        "请在本地终端执行 `base64 -w0 ~/.config/rclone/rclone.conf`，"
        "将输出粘贴到下方输入框并点击按钮（不会阻塞或中断会话）："
    )
    display(widgets.VBox([widgets.HBox([password_box, submit_btn]), status_out]))


def setup_colab_rclone(
    *,
    rclone_remote: str,
    robot: str,
    backend: str,
    task: str,
    use_colab_secrets: bool = False,
) -> None:
    """Inspect GPU/RAM, install rclone if needed, configure auth, and verify remote access."""
    _print_gpu_status()
    _print_memory_status()
    ensure_rclone_installed()
    remote_name = _remote_name_from_spec(rclone_remote)

    loaded = _try_load_existing_conf(remote_name, use_colab_secrets=use_colab_secrets)
    if loaded is not None:
        verify_and_prepare_remote(rclone_remote, robot, backend, task)
        return

    _prompt_with_ipywidgets(
        remote_name=remote_name,
        rclone_remote=rclone_remote,
        robot=robot,
        backend=backend,
        task=task,
    )
