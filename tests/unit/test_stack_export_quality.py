

from dual_arm_runtime.offline import export_episode


def _event(sim_time_ns, pixels):
    return {
        "event": "backend.render.quality",
        "sim_time_ns": sim_time_ns,
        "fields": {
            "right_wrist_source_pixels": pixels,
            "left_wrist_source_pixels": 0,
        },
    }


def _rules(required=64, ratio=0.85, blackout=0.2, required_phases=None):
    return {
        "max_blackout_s": blackout,
        "min_qualified_ratio": ratio,
        "phases": ["pregrasp", "grasp", "stable"],
        "required_phases": list(required_phases or []),
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
    assert no_phase["qualified"] is False
    assert any("no phase boundary" in reason for reason in no_phase["failure_reasons"])
    assert no_phase["rules"][0]["phase"] == "global"


def test_missing_required_phase_fails_even_when_rules_pass():
    summary = export_episode.quality_summary(
        [_event(1_000_000_000, 80)],
        {"pregrasp": 1},
        {1: 1_000_000_000},
        _rules(required_phases=["dual_hold"]),
    )
    assert summary["qualified"] is False
    assert "missing required phase evidence: dual_hold" in summary["failure_reasons"]


def test_scenario_phase_names_are_used_in_order():
    phases = ["left_approach", "dual_hold", "stable"]
    events = [
        {
            "event": "follower.chunk_accepted",
            "sim_time_ns": 1_000_000_000,
            "fields": {"phase": "left_approach"},
        },
        {
            "event": "follower.chunk_accepted",
            "sim_time_ns": 2_000_000_000,
            "fields": {"phase": "dual_hold"},
        },
        {
            "event": "follower.chunk_accepted",
            "sim_time_ns": 3_000_000_000,
            "fields": {"phase": "pregrasp"},
        },
    ]
    observed = export_episode.phase_observations(
        events, {1: 1_000_000_000, 2: 2_000_000_000, 3: 3_000_000_000}, phases
    )
    assert observed == {"left_approach": 1, "dual_hold": 2}
