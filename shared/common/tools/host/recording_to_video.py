#!/usr/bin/env python3
"""Host-only CLI to render recorded inference/teacher MCAP episodes to MP4 videos."""

from __future__ import annotations

import argparse
import json
import math
import os
import shutil
import subprocess
import sys
import threading
import time
from pathlib import Path

from host.docker_pool import (
    ROOT,
    BaseDockerWorkers,
    ascii_bar,
    finish_progress_line,
    format_duration,
    host_path_to_container,
    write_progress_line,
)


def parser() -> argparse.ArgumentParser:
    cli = argparse.ArgumentParser(
        prog="./recording_to_video.sh",
        description="将录制的推理/采集文件夹（MCAP 多路相机）合成为可视化 MP4 视频。",
    )
    cli.add_argument(
        "input_dir",
        type=Path,
        help="要转换的录制文件夹（单局目录、recording/ 目录或包含多个录制子目录的批次目录）",
    )
    cli.add_argument(
        "-o",
        "--output",
        type=Path,
        default=None,
        help="输出 MP4 路径或目录（默认保存到每个录制目录同目录下的 <目录名>.mp4）",
    )
    cli.add_argument(
        "--fps",
        type=float,
        default=None,
        help="覆盖输出帧率（默认自动读取录制帧率）",
    )
    cli.add_argument(
        "--speed",
        type=float,
        default=1.0,
        help="视频播放倍速，范围 (0.1, 20.0]（默认: 1.0）",
    )
    cli.add_argument(
        "--cameras",
        default="",
        help="逗号分隔的相机名称列表（默认按录制观测顺序展示全部相机）",
    )
    cli.add_argument(
        "--no-overlay",
        action="store_true",
        help="关闭顶部状态栏与相机名称叠加层",
    )
    cli.add_argument(
        "--skip-existing",
        action="store_true",
        help="目标 MP4 已存在时跳过（默认覆盖更新）",
    )
    cli.add_argument(
        "--project-root",
        type=Path,
        default=ROOT,
        help="项目根目录（默认当前仓库根目录）",
    )
    return cli


def discover_episodes(input_dir: Path) -> list[tuple[Path, Path]]:
    """Return [(episode_dir_for_runtime, default_mp4_path), ...] from input_dir."""
    root = input_dir.resolve()
    if not root.exists():
        raise FileNotFoundError(f"输入目录不存在: {root}")
    if not root.is_dir():
        raise NotADirectoryError(f"输入路径不是目录: {root}")

    if (root / "recording" / "metadata.yaml").is_file():
        return [(root, root / f"{root.name}.mp4")]
    if (root / "raw" / "recording" / "metadata.yaml").is_file():
        return [(root / "raw", root / f"{root.name}.mp4")]
    if (root / "metadata.yaml").is_file():
        return [(root, root / f"{root.name}.mp4")]

    found: list[tuple[Path, Path]] = []
    seen_episodes: set[Path] = set()
    for meta in sorted(root.rglob("recording/metadata.yaml")):
        if any(part.startswith(".partial-") for part in meta.parts):
            continue
        episode_dir = meta.parent.parent
        if episode_dir in seen_episodes:
            continue
        seen_episodes.add(episode_dir)
        if episode_dir.name == "raw" and episode_dir.parent != root:
            default_out = episode_dir.parent / f"{episode_dir.parent.name}.mp4"
        else:
            default_out = episode_dir / f"{episode_dir.name}.mp4"
        found.append((episode_dir, default_out))

    if not found:
        raise FileNotFoundError(
            f"目录 {root} 下未发现有效录制（未找到 recording/metadata.yaml）。\n"
            "请传入单局录制目录（如 output/<robot>/<backend>/<task>/logs/recordings/<run-id>）"
            "或批量录制目录（如 output/<robot>/<backend>/<task>/logs/batch-policy/<batch-id>）。"
        )
    return found


def resolve_outputs(
    episodes: list[tuple[Path, Path]], output: Path | None
) -> list[tuple[Path, Path]]:
    """Resolve final MP4 output path for every discovered episode."""
    if output is None:
        return episodes
    resolved = output.resolve()
    if len(episodes) == 1:
        ep_dir, default_mp4 = episodes[0]
        if resolved.is_dir() or (not resolved.exists() and resolved.suffix.lower() != ".mp4"):
            return [(ep_dir, resolved / default_mp4.name)]
        return [(ep_dir, resolved)]
    if resolved.exists() and not resolved.is_dir():
        raise ValueError(f"批量转换多个录制时，--output 必须是目录而非文件: {resolved}")
    used_names: dict[str, int] = {}
    targets: list[tuple[Path, Path]] = []
    for ep_dir, default_mp4 in episodes:
        stem = default_mp4.stem
        count = used_names.get(stem, 0) + 1
        used_names[stem] = count
        filename = f"{stem}.mp4" if count == 1 else f"{stem}_{count}.mp4"
        targets.append((ep_dir, resolved / filename))
    return targets


class _RuntimeVideoRunner(BaseDockerWorkers):
    def __init__(self, project_root: Path) -> None:
        super().__init__(
            argparse.Namespace(project_root=project_root),
            project_root / "output" / "_system",
            batch_name="video",
        )

    def exec_command(self, arguments: list[str]) -> list[str]:
        shell = "source /shared/common/tools/docker/setup.sh; exec \"$@\""
        return [
            "docker",
            "exec",
            "-i",
            self.runtime,
            "bash",
            "-c",
            shell,
            "bash",
            *arguments,
        ]


def _inspect_episode(
    runner: _RuntimeVideoRunner,
    episode_dir: Path,
    project_root: Path,
    cameras: str,
    no_overlay: bool,
) -> dict:
    container_dir = host_path_to_container(episode_dir, project_root)
    cmd = [
        "python3",
        "-m",
        "dual_arm_runtime.offline.video",
        "inspect",
        container_dir,
    ]
    if cameras:
        cmd += ["--cameras", cameras]
    if no_overlay:
        cmd.append("--no-overlay")
    result = subprocess.run(
        runner.exec_command(cmd),
        check=False,
        capture_output=True,
        text=True,
        timeout=60,
    )
    if result.returncode != 0:
        detail = (result.stderr or result.stdout).strip()
        raise RuntimeError(f"读取录制元信息失败 ({episode_dir}): {detail}")
    return json.loads(result.stdout)


def _convert_one(
    runner: _RuntimeVideoRunner,
    episode_dir: Path,
    out_mp4: Path,
    info: dict,
    *,
    project_root: Path,
    fps_override: float | None,
    speed: float,
    cameras: str,
    no_overlay: bool,
    idx: int,
    total: int,
    started: float,
    is_tty: bool,
) -> None:
    container_dir = host_path_to_container(episode_dir, project_root)
    base_fps = float(fps_override if fps_override is not None else info["fps"])
    out_fps = round(base_fps * speed, 3)
    width = int(info["width"])
    height = int(info["height"])
    total_frames = int(info.get("total_frames") or 0)

    out_mp4.parent.mkdir(parents=True, exist_ok=True)
    tmp_mp4 = out_mp4.with_name(f".{out_mp4.stem}.tmp_{os.getpid()}.mp4")
    tmp_mp4.unlink(missing_ok=True)

    stream_cmd = [
        "python3",
        "-m",
        "dual_arm_runtime.offline.video",
        "stream",
        container_dir,
    ]
    if cameras:
        stream_cmd += ["--cameras", cameras]
    if no_overlay:
        stream_cmd.append("--no-overlay")

    ffmpeg_cmd = [
        "ffmpeg",
        "-y",
        "-hide_banner",
        "-loglevel",
        "error",
        "-f",
        "rawvideo",
        "-pix_fmt",
        "rgb24",
        "-s",
        f"{width}x{height}",
        "-r",
        f"{out_fps}",
        "-i",
        "pipe:0",
        "-c:v",
        "libx264",
        "-preset",
        "fast",
        "-crf",
        "20",
        "-pix_fmt",
        "yuv420p",
        "-movflags",
        "+faststart",
        str(tmp_mp4),
    ]

    producer: subprocess.Popen | None = None
    encoder: subprocess.Popen | None = None
    ffmpeg_errors: list[str] = []
    producer_errors: list[str] = []
    label = out_mp4.stem[:14]

    try:
        producer = subprocess.Popen(
            runner.exec_command(stream_cmd),
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
        )
        assert producer.stdout is not None
        assert producer.stderr is not None
        encoder = subprocess.Popen(
            ffmpeg_cmd,
            stdin=producer.stdout,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.PIPE,
        )
        producer.stdout.close()

        def _drain_ffmpeg() -> None:
            assert encoder is not None and encoder.stderr is not None
            data = encoder.stderr.read().decode("utf-8", errors="replace").strip()
            if data:
                ffmpeg_errors.append(data)

        drain_thread = threading.Thread(target=_drain_ffmpeg, daemon=True)
        drain_thread.start()

        current_done = 0
        current_total = max(1, total_frames)
        for raw_line in producer.stderr:
            text = raw_line.decode("utf-8", errors="replace").strip()
            if text.startswith("PROGRESS "):
                parts = text.split()
                if len(parts) == 3 and parts[1].isdigit() and parts[2].isdigit():
                    current_done = int(parts[1])
                    current_total = max(1, int(parts[2]))
                    if is_tty:
                        elapsed = time.monotonic() - started
                        overall = (idx - 1 + min(1.0, current_done / current_total)) / total
                        bar = ascii_bar(round(overall * 100), 100, width=10)
                        eta = (
                            elapsed * (1.0 - overall) / overall
                            if overall > 0.02
                            else None
                        )
                        line = (
                            f"转换视频 [{bar}] {idx}/{total} | {label} "
                            f"{current_done}/{current_total}帧 | "
                            f"用时 {format_duration(elapsed)} | 剩余 {format_duration(eta)}"
                        )
                        write_progress_line(line, tty=True)
            elif text:
                producer_errors.append(text)

        prod_code = producer.wait()
        enc_code = encoder.wait()
        drain_thread.join(timeout=5)

        if prod_code != 0 or enc_code != 0 or not tmp_mp4.is_file() or tmp_mp4.stat().st_size == 0:
            details = "; ".join([*producer_errors, *ffmpeg_errors]) or f"producer={prod_code}, ffmpeg={enc_code}"
            raise RuntimeError(f"视频合成失败 ({episode_dir.name}): {details}")

        os.replace(tmp_mp4, out_mp4)
        if not is_tty:
            elapsed = time.monotonic() - started
            write_progress_line(
                f"转换视频 [{idx}/{total}] {out_mp4} ({current_done}帧, 用时 {format_duration(elapsed)})",
                tty=False,
            )
    except BaseException:
        for proc in (encoder, producer):
            if proc is not None and proc.poll() is None:
                proc.terminate()
                try:
                    proc.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    proc.kill()
        tmp_mp4.unlink(missing_ok=True)
        raise


def main(argv: list[str] | None = None) -> int:
    cli = parser()
    args = cli.parse_args(argv)
    if args.fps is not None and (not math.isfinite(args.fps) or args.fps <= 0):
        cli.error("--fps 必须是大于 0 的有限数值")
    if not math.isfinite(args.speed) or not 0.1 < args.speed <= 20.0:
        cli.error("--speed 必须位于 (0.1, 20.0] 范围内")
    if shutil.which("ffmpeg") is None:
        print(
            "错误: 宿主机未检测到 ffmpeg，请先安装（例如: sudo apt install ffmpeg）",
            file=sys.stderr,
        )
        return 2

    project_root = args.project_root.resolve()
    try:
        episodes = discover_episodes(args.input_dir)
        targets = resolve_outputs(episodes, args.output)
        for ep_dir, _ in targets:
            host_path_to_container(ep_dir, project_root)
    except (OSError, ValueError) as error:
        print(f"错误: {error}", file=sys.stderr)
        return 2

    runner = _RuntimeVideoRunner(project_root)
    try:
        runner.ensure_runtime()
        first_info = _inspect_episode(
            runner,
            targets[0][0],
            project_root,
            args.cameras,
            args.no_overlay,
        )
        effective_fps = round(
            float(args.fps if args.fps is not None else first_info["fps"]) * args.speed, 3
        )
        print("=== 录制转视频可视化 ===", flush=True)
        print(f"输入路径: {args.input_dir.resolve()}", flush=True)
        print(
            f"匹配局数: {len(targets)} | 相机: {', '.join(first_info['cameras'])} | "
            f"分辨率: {first_info['width']}x{first_info['height']} | "
            f"输出帧率: {effective_fps} fps（倍速 {args.speed}x）",
            flush=True,
        )
        if len(targets) == 1:
            print(f"输出文件: {targets[0][1]}", flush=True)
        else:
            out_desc = (
                str(args.output.resolve())
                if args.output is not None
                else "各录制目录同级目录下 (<目录名>.mp4)"
            )
            print(f"输出位置: {out_desc}", flush=True)

        is_tty = sys.stdout.isatty()
        started = time.monotonic()
        converted = 0
        skipped = 0
        try:
            for idx, (ep_dir, out_mp4) in enumerate(targets, start=1):
                if args.skip_existing and out_mp4.is_file() and out_mp4.stat().st_size > 0:
                    skipped += 1
                    continue
                info = (
                    first_info
                    if idx == 1
                    else _inspect_episode(
                        runner, ep_dir, project_root, args.cameras, args.no_overlay
                    )
                )
                _convert_one(
                    runner,
                    ep_dir,
                    out_mp4,
                    info,
                    project_root=project_root,
                    fps_override=args.fps,
                    speed=args.speed,
                    cameras=args.cameras,
                    no_overlay=args.no_overlay,
                    idx=idx,
                    total=len(targets),
                    started=started,
                    is_tty=is_tty,
                )
                converted += 1
        finally:
            if is_tty and converted > 0:
                finish_progress_line(tty=True)

        elapsed = time.monotonic() - started
        print(
            f"转换完成：成功 {converted}/{len(targets)} | 跳过 {skipped} | "
            f"用时 {format_duration(elapsed)}",
            flush=True,
        )
        if len(targets) == 1 and converted == 1:
            print(f"MP4 文件: {targets[0][1]}", flush=True)
        return 0
    except KeyboardInterrupt:
        print("\n收到中断信号，已取消视频转换。", file=sys.stderr, flush=True)
        return 130
    except (OSError, RuntimeError, ValueError, subprocess.SubprocessError) as error:
        print(f"错误: {error}", file=sys.stderr, flush=True)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
