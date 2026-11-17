"""Focused tests for static-anchor sampling and scenario-agnostic dual-arm motion."""

from __future__ import annotations

import json
from pathlib import Path

import numpy as np
import pytest
from model_helpers import replace

from dual_arm_learning.config import load_training_config
from dual_arm_learning.dataset import canonical_hash, file_sha256
from dual_arm_learning.sample_filter import (
    FilterError,
    build_filter_manifest,
    build_filter_mask,
    load_filter_mask,
    load_filter_rule,
)
from dual_arm_learning.training import TrainingError, _sample_filter

pa = pytest.importorskip("pyarrow")
pq = pytest.importorskip("pyarrow.parquet")
ROOT = Path(__file__).resolve().parents[2]
SCENARIO = ROOT / "shared/scenarios/panda_sim/stack"
RULE = SCENARIO / "configs/learning/act_sample_filter.yaml"
PROFILE = "a" * 64
LEAD_NS = 50_000_000


def _write_json(path: Path, value: object) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, indent=2) + "\n", encoding="utf-8")


def _spec(
    frames: int,
    *,
    phases: list[str] | None = None,
    gates: list[str] | None = None,
    measured: np.ndarray | None = None,
    applied: np.ndarray | None = None,
    measured_velocity: np.ndarray | None = None,
    applied_velocity: np.ndarray | None = None,
) -> dict[str, object]:
    zeros = np.zeros((frames, 16), dtype=np.float64)
    return {
        "frames": frames,
        "phases": phases or ["stable"] * frames,
        "gates": gates or ["TRACKING"] * frames,
        "measured": zeros.copy() if measured is None else measured,
        "applied": zeros.copy() if applied is None else applied,
        "measured_velocity": (zeros.copy() if measured_velocity is None else measured_velocity),
        "applied_velocity": zeros.copy() if applied_velocity is None else applied_velocity,
    }


def _events(
    attempt_dir: Path, attempt_id: str, sequences: list[int], phase_gate: dict[int, tuple[str, str]]
) -> None:
    runtime = attempt_dir / "logs" / f"run-{attempt_id}" / "logs" / "runtime" / "instance"
    path = runtime / "events.000001.jsonl"
    path.parent.mkdir(parents=True, exist_ok=True)
    lines = []
    for chunk_seq in sorted(set(sequences)):
        phase, gate = phase_gate[chunk_seq]
        lines.append(
            json.dumps(
                {
                    "event": "follower.chunk_accepted",
                    "fields": {
                        "chunk_seq": chunk_seq,
                        "phase": phase,
                        "gate_state": gate,
                    },
                }
            )
        )
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def _write_split(
    root: Path,
    name: str,
    specs: list[dict[str, object]],
    *,
    first_global: int,
) -> tuple[dict[str, object], list[dict[str, object]]]:
    split_root = root / name
    parquet_path = split_root / "data/chunk-000/file-000.parquet"
    parquet_path.parent.mkdir(parents=True, exist_ok=True)
    columns: dict[str, list[object]] = {
        "index": [],
        "episode_index": [],
        "source.episode_index": [],
        "source.seed": [],
        "observation.state": [],
        "observation.velocity": [],
        "action": [],
        "observation.sim_time_ns": [],
        "action.sim_time_ns": [],
    }
    manifest_episodes: list[dict[str, object]] = []
    next_index = 0
    for local, spec in enumerate(specs):
        attempt_id = f"{name}-{local:03d}"
        seed = first_global + local
        global_index = first_global + local
        frames = int(spec["frames"])
        measured = np.asarray(spec["measured"])
        applied = np.asarray(spec["applied"])
        measured_velocity = np.asarray(spec["measured_velocity"])
        applied_velocity = np.asarray(spec["applied_velocity"])
        phases = list(spec["phases"])
        gates = list(spec["gates"])
        start_ns = 1_000_000_000 + global_index * 10_000_000_000
        observation_times = [start_ns + index * LEAD_NS for index in range(frames)]
        sequences: list[int] = []
        phase_gate: dict[int, tuple[str, str]] = {}
        chunk_seq = 0
        previous = None
        for phase, gate in zip(phases, gates, strict=True):
            if (phase, gate) != previous:
                chunk_seq += 1
                previous = (phase, gate)
                phase_gate[chunk_seq] = (phase, gate)
            sequences.append(chunk_seq)
        labels = []
        for index in range(frames):
            labels.append(
                {
                    "observation_id": index + 1,
                    "state_seq": index + 1,
                    "observation_sim_time_ns": observation_times[index],
                    "label_sim_time_ns": observation_times[index] + LEAD_NS,
                    "observation_period_ns": LEAD_NS,
                    "chunk_seq": sequences[index],
                    "raw_chunk_seq": sequences[index],
                    "measured_positions": measured[index].tolist(),
                    "applied_positions": applied[index].tolist(),
                    "measured_velocities": measured_velocity[index].tolist(),
                    "applied_velocities": applied_velocity[index].tolist(),
                }
            )
        attempt_dir = root / "attempts" / attempt_id
        export = attempt_dir / "export"
        _write_json(
            export / "labels.json",
            {
                "schema_version": 1,
                "episode_id": attempt_id,
                "robot_profile_hash": PROFILE,
                "seed": seed,
                "labels": labels,
            },
        )
        _events(attempt_dir, attempt_id, sequences, phase_gate)
        for index in range(frames):
            columns["index"].append(next_index)
            columns["episode_index"].append(local)
            columns["source.episode_index"].append(global_index)
            columns["source.seed"].append(seed)
            columns["observation.state"].append(measured[index].tolist())
            columns["observation.velocity"].append(measured_velocity[index].tolist())
            columns["action"].append(applied[index].tolist())
            columns["observation.sim_time_ns"].append(observation_times[index])
            columns["action.sim_time_ns"].append(observation_times[index] + LEAD_NS)
            next_index += 1
        manifest_episodes.append(
            {
                "dataset_episode_index": local,
                "global_success_index": global_index,
                "attempt_id": attempt_id,
                "seed": seed,
                "attempt_dir": str(attempt_dir),
                "frame_count": frames,
            }
        )
    table = pa.table({key: pa.array(value) for key, value in columns.items()})
    pq.write_table(table, parquet_path)
    relative = parquet_path.relative_to(split_root).as_posix()
    split = {
        "root": name,
        "episode_count": len(specs),
        "frame_count": len(columns["index"]),
        "episodes": manifest_episodes,
        "files": {
            relative: {
                "sha256": file_sha256(parquet_path),
                "bytes": parquet_path.stat().st_size,
            }
        },
    }
    return split, manifest_episodes


def _make_source(
    root: Path,
    *,
    training: list[dict[str, object]] | None = None,
    validation: list[dict[str, object]] | None = None,
) -> Path:
    training = training or [_spec(80)]
    validation = validation or [_spec(20)]
    root.mkdir(parents=True)
    train_split, _ = _write_split(root, "train", training, first_global=0)
    validation_split, _ = _write_split(root, "validation", validation, first_global=1000)
    manifest = {
        "schema_version": 1,
        "status": "complete",
        "synthetic": False,
        "profile": {"semantic_sha256": PROFILE},
        "lerobot": {"action_lead_ns": LEAD_NS, "fps": 20, "chunk_size": 60},
        "splits": {"training": train_split, "validation": validation_split},
    }
    _write_json(root / "manifest.json", manifest)
    return root


def _build(
    root: Path,
    *,
    training: list[dict[str, object]] | None = None,
    validation: list[dict[str, object]] | None = None,
) -> tuple[dict[str, object], Path]:
    dataset_root = _make_source(root, training=training, validation=validation)
    document = build_filter_manifest(rule_path=RULE, dataset_root=dataset_root)
    return document, dataset_root


def _refresh_parquet_hash(root: Path, split_name: str) -> None:
    manifest_path = root / "manifest.json"
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    split = manifest["splits"][split_name]
    relative = next(iter(split["files"]))
    parquet_path = root / split["root"] / relative
    split["files"][relative] = {
        "sha256": file_sha256(parquet_path),
        "bytes": parquet_path.stat().st_size,
    }
    _write_json(manifest_path, manifest)


def test_real_filter_rule_loads():
    rule = load_filter_rule(RULE)
    assert rule.arm_indices == (0, 1, 2, 3, 4, 5, 6, 8, 9, 10, 11, 12, 13, 14)
    assert rule.gripper_indices == (7, 15)
    assert rule.min_static_run_frames == 15
    assert rule.max_arm_step_rad == 0.001


def test_left_arm_idle_but_right_arm_motion_is_not_filtered(tmp_path):
    spec = _spec(80)
    step = np.arange(80, dtype=np.float64)[:, None] * 0.01
    spec["measured"][:, 8:15] = step
    spec["applied"][:, 8:15] = step
    spec["measured_velocity"][:, 8:15] = 0.2
    spec["applied_velocity"][:, 8:15] = 0.2
    document, _ = _build(tmp_path / "data", training=[spec])
    assert document["training"]["excluded_frames"] == 0
    assert document["training"]["episodes"][0]["kept_count"] == 80


def test_right_arm_idle_but_left_arm_motion_is_not_filtered(tmp_path):
    """The old right-arm-only rule mistakenly treated a moving left arm as static."""
    spec = _spec(80)
    step = np.arange(80, dtype=np.float64)[:, None] * 0.01
    spec["measured"][:, 0:7] = step
    spec["applied"][:, 0:7] = step
    spec["measured_velocity"][:, 0:7] = 0.2
    spec["applied_velocity"][:, 0:7] = 0.2
    document, _ = _build(tmp_path / "data", training=[spec])
    assert document["training"]["excluded_frames"] == 0
    assert document["training"]["episodes"][0]["kept_count"] == 80


def test_machine_static_but_gripper_motion_is_not_filtered(tmp_path):
    spec = _spec(80)
    spec["measured"][:, 7] = np.arange(80, dtype=np.float64) * 0.01
    spec["applied"][:, 15] = -np.arange(80, dtype=np.float64) * 0.01
    spec["measured_velocity"][:, 7] = 0.2
    spec["applied_velocity"][:, 15] = 0.2
    document, _ = _build(tmp_path / "data", training=[spec])
    assert document["training"]["excluded_frames"] == 0
    assert document["training"]["episodes"][0]["kept_count"] == 80


def test_short_static_pause_is_preserved_completely(tmp_path):
    positions = np.zeros(60, dtype=np.float64)
    positions[:31] = np.arange(31, dtype=np.float64) * 0.01
    positions[31:41] = 0.30
    positions[41:] = 0.30 + np.arange(1, 20, dtype=np.float64) * 0.01
    spec = _spec(60)
    spec["measured"][:, 8] = positions
    spec["applied"][:, 8] = positions
    document, _ = _build(tmp_path / "data", training=[spec])
    assert document["training"]["excluded_frames"] == 0
    assert document["training"]["episodes"][0]["static_runs"]["long"] == 0


def test_long_static_run_keeps_edges_and_every_fifth_anchor(tmp_path):
    document, _ = _build(tmp_path / "data", training=[_spec(300)])
    episode = document["training"]["episodes"][0]
    expected = set(range(5)) | set(range(295, 300)) | set(range(5, 295, 5))
    assert set(episode["kept_indices"]) == expected
    assert episode["kept_count"] == 68
    assert episode["excluded_count"] == 232
    assert episode["excluded_reasons"] == {"redundant_static_anchor": 232}
    assert episode["fallback"] is None


def test_long_static_holds_are_thinned_while_transitions_are_kept(tmp_path):
    """WAITING_FOR_FEEDBACK/HOLDING holds and unknown phases are no longer blanket-kept."""
    phases = (
        ["stable"] * 60 + ["transport"] * 60 + ["open"] * 60 + ["stable"] * 60 + ["mystery"] * 60
    )
    gates = (
        ["TRACKING"] * 60
        + ["TRACKING"] * 60
        + ["WAITING_FOR_FEEDBACK"] * 60
        + ["TRACKING"] * 60
        + ["TRACKING"] * 60
    )
    document, _ = _build(tmp_path / "data", training=[_spec(300, phases=phases, gates=gates)])
    episode = document["training"]["episodes"][0]
    kept = set(episode["kept_indices"])
    excluded = set(episode["excluded_indices"])
    for start, end in ((55, 65), (115, 125), (175, 185), (235, 245)):
        assert set(range(start, end + 1)) <= kept
    # The 60-frame physics wait and the unknown-phase hold must lose interior frames.
    assert excluded & set(range(126, 174))
    assert excluded & set(range(246, 294))
    assert document["training"]["excluded_frames"] > 0
    assert document["training"]["gate_counts"]["source"]["WAITING_FOR_FEEDBACK"] == 60
    assert document["training"]["gate_counts"]["source"].get("unknown", 0) == 0
    assert document["training"]["phase_counts"]["source"]["mystery"] == 60


def test_validation_split_is_never_filtered(tmp_path):
    document, dataset_root = _build(tmp_path / "data", training=[_spec(300)])
    mask = build_filter_mask(rule_path=RULE, dataset_root=dataset_root)
    assert document["validation"]["filtered"] is False
    assert document["validation"]["excluded_frames"] == 0
    assert document["validation"]["kept_frames"] == document["validation"]["source_frames"]
    assert document["training"]["excluded_frames"] > 0
    assert document["validation"]["kept_frames"] == 20
    assert len(mask.eligible_indices) == document["training"]["eligible_frames"]
    assert max(mask.eligible_indices) < document["training"]["source_frames"]
    assert mask.source == RULE.resolve()


def test_source_manifest_hash_and_duplicate_index_are_rejected(tmp_path):
    bad_manifest = tmp_path / "bad-manifest"
    root = _make_source(bad_manifest)
    manifest_path = root / "manifest.json"
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    relative = next(iter(manifest["splits"]["training"]["files"]))
    manifest["splits"]["training"]["files"][relative]["sha256"] = "0" * 64
    _write_json(manifest_path, manifest)
    with pytest.raises(FilterError, match="hash"):
        build_filter_manifest(rule_path=RULE, dataset_root=root)

    duplicate = tmp_path / "duplicate-index"
    root = _make_source(duplicate)
    parquet_path = root / "train/data/chunk-000/file-000.parquet"
    rows = pq.read_table(parquet_path).to_pydict()
    rows["index"][1] = rows["index"][0]
    pq.write_table(pa.table({key: pa.array(value) for key, value in rows.items()}), parquet_path)
    _refresh_parquet_hash(root, "training")
    with pytest.raises(FilterError, match="duplicate"):
        build_filter_manifest(rule_path=RULE, dataset_root=root)

    out_of_range = tmp_path / "out-of-range"
    root = _make_source(out_of_range)
    parquet_path = root / "train/data/chunk-000/file-000.parquet"
    rows = pq.read_table(parquet_path).to_pydict()
    rows["index"][0] = len(rows["index"])
    pq.write_table(pa.table({key: pa.array(value) for key, value in rows.items()}), parquet_path)
    _refresh_parquet_hash(root, "training")
    with pytest.raises(FilterError, match="out of range"):
        build_filter_manifest(rule_path=RULE, dataset_root=root)


def test_mask_digest_and_index_structure_are_rejected(tmp_path):
    document, _ = _build(tmp_path / "data")
    document["training"]["episodes"][0]["kept_indices"][-1] = 999_999
    payload = {
        key: value
        for key, value in document.items()
        if key not in {"filter_digest", "created_wall_time_ns"}
    }
    document["filter_digest"] = canonical_hash(payload)
    mask_path = tmp_path / "tampered.json"
    _write_json(mask_path, document)
    with pytest.raises(FilterError, match="partition"):
        load_filter_mask(mask_path)

    clean, _ = _build(tmp_path / "clean")
    clean["filter_digest"] = "0" * 64
    clean_path = tmp_path / "clean.json"
    _write_json(clean_path, clean)
    with pytest.raises(FilterError, match="digest"):
        load_filter_mask(clean_path)


def test_future_action_chunks_keep_their_original_interval_and_validation_is_unfiltered():
    torch = pytest.importorskip("torch")
    from dual_arm_learning import training

    class _Dataset:
        def __len__(self):
            return 20

        def __getitem__(self, index):
            return {
                "anchor": index,
                "observation.state": torch.zeros(16),
                "action": torch.full((60, 16), float(index)),
                "action_is_pad": torch.zeros(60, dtype=torch.bool),
            }

    config = replace(
        load_training_config(SCENARIO / "configs/learning/act_training.yaml", ROOT),
        batch_size=2,
        num_workers=0,
    )
    loader = training._loader(_Dataset(), config, training=True, indices=[2, 7, 19])
    batches = list(loader)
    anchors = sorted(int(value) for batch in batches for value in batch["anchor"])
    assert anchors == [2, 7, 19]
    for batch in batches:
        assert batch["action"].shape[1:] == (60, 16)
        for row, anchor in enumerate(batch["anchor"]):
            assert torch.equal(batch["action"][row], torch.full((60, 16), float(anchor)))
    validation = training._loader(_Dataset(), config, training=False)
    assert [
        int(value) for batch in validation for value in batch["anchor"]
    ] == list(range(20))


def test_sample_filter_is_configured_for_full_mode_only(tmp_path):
    config = load_training_config(SCENARIO / "configs/learning/act_training.yaml", ROOT)
    assert config.sample_filter == RULE.resolve()
    assert _sample_filter(config, None) is None
    unsupported = config.model_copy(
        update={"mode": "full", "sample_filter": tmp_path / "filter.txt"}
    )
    with pytest.raises(TrainingError, match="rule YAML or a mask JSON"):
        _sample_filter(unsupported, None)


def test_mask_json_can_be_loaded_directly(tmp_path):
    document, _ = _build(tmp_path / "data")
    path = tmp_path / "mask.json"
    _write_json(path, document)
    mask = load_filter_mask(path, verify_source=False)
    assert mask.source == path.resolve()
    assert len(mask.eligible_indices) == document["training"]["eligible_frames"]
    assert mask.source_sha256 == file_sha256(path)
