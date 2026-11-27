"""Unit tests for MCAP recording-to-video layout, discovery, and host CLI."""

from __future__ import annotations

import os
import subprocess
from pathlib import Path

import pytest
from host import recording_to_video

ROOT = Path(__file__).resolve().parents[3]


def _touch_bag(bag_dir: Path) -> None:
    bag_dir.mkdir(parents=True, exist_ok=True)
    (bag_dir / "metadata.yaml").write_text("rosbag2_bagfile_information: {}\n", encoding="utf-8")
    (bag_dir / "recording_0.mcap").write_bytes(b"mcap")


def test_discover_episodes_handles_single_raw_direct_and_batch(tmp_path: Path):
    single = tmp_path / "policy_run"
    _touch_bag(single / "recording")
    assert recording_to_video.discover_episodes(single) == [
        (single, single / "policy_run.mp4")
    ]

    attempt = tmp_path / "formal-00001-worker"
    _touch_bag(attempt / "raw" / "recording")
    assert recording_to_video.discover_episodes(attempt) == [
        (attempt / "raw", attempt / "formal-00001-worker.mp4")
    ]

    direct_bag = tmp_path / "direct_bag"
    _touch_bag(direct_bag)
    assert recording_to_video.discover_episodes(direct_bag) == [
        (direct_bag, direct_bag / "direct_bag.mp4")
    ]

    batch = tmp_path / "batch_01"
    _touch_bag(batch / "ep0001" / "recording")
    _touch_bag(batch / "ep0002" / "recording")
    _touch_bag(batch / ".partial-ep0003" / "recording")
    discovered = recording_to_video.discover_episodes(batch)
    assert discovered == [
        (batch / "ep0001", batch / "ep0001" / "ep0001.mp4"),
        (batch / "ep0002", batch / "ep0002" / "ep0002.mp4"),
    ]


def test_resolve_outputs_preserves_default_and_handles_overrides(tmp_path: Path):
    ep1 = tmp_path / "ep1"
    ep2 = tmp_path / "ep2"
    pairs = [(ep1, ep1 / "ep1.mp4"), (ep2, ep2 / "ep2.mp4")]

    assert recording_to_video.resolve_outputs([pairs[0]], None) == [pairs[0]]
    assert recording_to_video.resolve_outputs([pairs[0]], tmp_path / "custom.mp4") == [
        (ep1, (tmp_path / "custom.mp4").resolve())
    ]

    out_dir = tmp_path / "videos"
    assert recording_to_video.resolve_outputs(pairs, out_dir) == [
        (ep1, (out_dir / "ep1.mp4").resolve()),
        (ep2, (out_dir / "ep2.mp4").resolve()),
    ]

    existing_file = tmp_path / "file.mp4"
    existing_file.write_bytes(b"x")
    with pytest.raises(ValueError, match="必须是目录"):
        recording_to_video.resolve_outputs(pairs, existing_file)


def test_video_layout_and_frame_composition_handles_even_dimensions_and_missing_cam():
    from dual_arm_runtime.offline import video

    for cam_count in (1, 2, 3, 4, 7):
        cams = tuple(f"cam_{i}" for i in range(cam_count))
        sizes = {cam: (321, 241) for cam in cams}
        layout = video.compute_layout(cams, sizes, overlay=True)
        assert layout.width % 2 == 0
        assert layout.height % 2 == 0
        assert layout.width >= layout.cols * 321
        assert layout.height >= layout.hud_height + layout.rows * (layout.label_height + 241)

    layout = video.compute_layout(
        ("cam_high", "cam_left_wrist", "cam_right_wrist"),
        {"cam_high": (320, 240), "cam_left_wrist": (320, 240), "cam_right_wrist": (320, 240)},
        overlay=True,
    )
    frame_bytes = video.compose_frame(
        layout,
        {"cam_high": (320, 240, b"\x10\x20\x30" * (320 * 240))},
        frame_index=1,
        total_frames=10,
        rel_time_s=0.05,
        fps=20.0,
        metadata={"episode_id": "ep-test", "action_source": "policy", "seed": 42},
        task_state={"phase": "RUNNING", "succeeded": False, "terminal": False, "cond_passed": 3, "cond_total": 10},
        fonts=video._load_fonts(),
    )
    assert len(frame_bytes) == layout.width * layout.height * 3


def test_recording_to_video_shell_rejects_container_execution():
    result = subprocess.run(
        ["bash", str(ROOT / "recording_to_video.sh"), "--help"],
        env={**os.environ, "PROJECT_CONTAINER": "1"},
        capture_output=True,
        text=True,
        timeout=10,
    )
    assert result.returncode == 2
    assert "宿主机" in result.stderr
