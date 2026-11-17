import json
from pathlib import Path
from types import SimpleNamespace

import pytest
from dual_arm_runtime.offline import export_episode
from dual_arm_runtime.offline.audit import label_export
from robot_contracts.profiles import load_profile

ROOT = Path(__file__).resolve().parents[2]
PROFILE = load_profile(
    ROOT / "shared/scenarios/panda_sim/stack/profiles/robots/panda_stack_v1/profile.json",
    ROOT / "shared/common/interfaces/schemas",
)


class _Reference:
    def __init__(self, start, end):
        self.start_sim_time_ns = start
        self.end_sim_time_ns = end
        self.request = SimpleNamespace(observation_time_ns=start)

    def applied_sample_at(self, sim_time_ns, profile):
        return SimpleNamespace(
            applied_positions=(0.04,) * 16,
            sampled_positions=(0.041,) * 16,
            sampled_velocities=(0.0,) * 16,
        )


def _stamp(seconds):
    return SimpleNamespace(sec=seconds // 1_000_000_000, nanosec=seconds % 1_000_000_000)


def _state(sequence, time_ns):
    return SimpleNamespace(
        state_seq=sequence,
        stamp=_stamp(time_ns),
        joints=SimpleNamespace(position=[0.0] * 16, velocity=[0.0] * 16),
    )


def _frame(sequence, time_ns):
    return SimpleNamespace(
        frame_seq=sequence,
        image=SimpleNamespace(header=SimpleNamespace(stamp=_stamp(time_ns))),
    )


def test_rebuilds_20hz_frames_with_4hz_raw_predictions():
    cameras = ("global", "left_wrist", "right_wrist")
    states = {}
    frames = {}
    for index in range(52):
        sequence = index + 1
        time_ns = 1_000_000_000 + index * 50_000_000
        states[sequence] = _state(sequence, time_ns)
        for camera in cameras:
            frames[(camera, sequence)] = _frame(sequence, time_ns)
    references = {}
    chunk_events = {}
    requests = {}
    for chunk in range(1, 9):
        start = 1_000_000_000 + (chunk - 1) * 250_000_000
        references[("command", chunk)] = _Reference(start, start + 250_000_000)
        chunk_events[("command", chunk)] = {
            "STARTED": start + 10_000_000,
            "RETIRED": start + 250_000_000,
        }
        source_observation = 1 + (chunk - 1) * 5
        times = tuple((i + 1) * 50_000_000 for i in range(60))
        requests[source_observation] = (
            "command",
            chunk,
            times,
            tuple((0.1,) * 16 for _ in times),
        )
    metadata = {
        "states": states,
        "frames": frames,
        "indices": {},
        "requests": requests,
        "references": references,
        "chunk_events": chunk_events,
        "brakes": [],
        "camera_names": cameras,
    }
    rows, raw, summary, times, frame_map = export_episode._build(
        metadata, cameras, [], 700, PROFILE
    )
    assert len(rows) > 30
    assert summary["rejected"]["missing_raw_block"] == 0
    assert len(raw) == 8
    assert len(frame_map) == 52
    assert rows[0]["capture_source"] == "state_rebuild"
    assert rows[0]["raw_source_observation_id"] in {1, 6, 11, 16, 21, 26, 31, 36}


@pytest.fixture(autouse=True)
def sample_test_reference(monkeypatch):
    """Explicit test_only sampler for the synthetic reference used in these label tests."""
    monkeypatch.setattr(
        label_export,
        "applied_follower_sample",
        lambda reference, stamp, profile: reference.applied_sample_at(stamp, profile),
    )


def test_write_documents_emits_three_json_files_and_folds_source_facts(tmp_path):
    out = tmp_path / "export"
    manifest = {
        "episode_id": "episode",
        "task": {"outcome": "succeeded", "reason": "TASK_SUCCEEDED"},
        "runtime": {"outcome": "stopped", "reason": "TASK_SUCCEEDED"},
        "recording": {"complete": True, "failure_reason": ""},
    }
    export_episode._write_documents(
        out,
        manifest,
        SimpleNamespace(digest="a" * 64),
        {"reset_seed": 7},
        {"action_source": "teacher"},
        [{"label_sim_time_ns": 50_000_000, "rgb": {}}],
        [],
        ("global", "left_wrist", "right_wrist"),
        {"labels": 1},
        {"qualified": True},
    )
    assert sorted(path.name for path in out.glob("*.json")) == [
        "labels.json",
        "quality.json",
        "raw_predictions.json",
    ]
    labels = json.loads((out / "labels.json").read_text())
    assert labels["camera_order"] == ["global", "left_wrist", "right_wrist"]
    assert labels["action_source"] == "teacher"
    assert labels["task"] == manifest["task"]
    assert labels["runtime"] == manifest["runtime"]
    assert labels["recording"] == manifest["recording"]
    assert labels["summary"] == {"labels": 1}
