import sys
from pathlib import Path

_TOOLS = Path(__file__).resolve().parents[2] / "runtime/ros2/scripts"
if str(_TOOLS) not in sys.path:
    sys.path.insert(0, str(_TOOLS))

import export_episode  # noqa: E402


def _event(sim_time_ns, pixels):
    return {
        "event": "backend.render.quality",
        "sim_time_ns": sim_time_ns,
        "fields": {
            "right_wrist_source_pixels": pixels,
            "left_wrist_source_pixels": 0,
        },
    }


def _rules(required=64, ratio=0.85, blackout=0.2):
    return {
        "max_blackout_s": blackout,
        "min_qualified_ratio": ratio,
        "rules": [
            {
                "camera": "right_wrist",
                "object": "source",
                "phases": ["pregrasp"],
                "mode": "required",
                "min_pixels": required,
            },
            {
                "camera": "left_wrist",
                "object": "source",
                "phases": ["pregrasp"],
                "mode": "record_only",
                "min_pixels": 64,
            },
        ],
    }


def test_quality_reports_record_only_without_gating_and_covers_full_phase():
    summary = export_episode.quality_summary(
        [
            _event(1_000_000_000, 80),
            _event(1_050_000_000, 80),
            _event(1_200_000_000, 80),
        ],
        {"pregrasp": 1},
        {1: 1_000_000_000, 2: 1_050_000_000},
        _rules(),
    )
    assert summary["qualified"] is True
    assert summary["phase_based"] is True
    assert summary["phases"]["pregrasp"]["rules"][1]["mode"] == "record_only"
    assert summary["phases"]["pregrasp"]["rules"][0]["samples"] == 3


def test_required_ratio_and_blackout_fail_but_record_only_does_not():
    summary = export_episode.quality_summary(
        [
            _event(1_000_000_000, 80),
            _event(1_050_000_000, 0),
            _event(1_200_000_000, 0),
        ],
        {"pregrasp": 1},
        {1: 1_000_000_000, 2: 1_050_000_000},
        _rules(),
    )
    assert summary["qualified"] is False
    assert any("blackout" in reason or "ratio" in reason for reason in summary["failure_reasons"])
    assert summary["phases"]["pregrasp"]["rules"][1]["qualified_ratio"] == 0.0


def test_missing_required_samples_or_phase_cannot_be_qualified():
    no_samples = export_episode.quality_summary(
        [], {"pregrasp": 1}, {1: 1_000_000_000}, _rules()
    )
    assert no_samples["qualified"] is False
    assert any("no right_wrist.source" in reason for reason in no_samples["failure_reasons"])

    no_phase = export_episode.quality_summary(
        [_event(1_000_000_000, 80)], {}, {1: 1_000_000_000}, _rules()
    )
    assert no_phase["phase_based"] is False
    assert no_phase["rules"][0]["phase"] == "global"
