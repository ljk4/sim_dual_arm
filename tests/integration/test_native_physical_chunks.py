"""No-ROS proof that the v3 evidence comes from real controller stepping."""

from __future__ import annotations

import pytest

pytest.importorskip("mujoco")
pytest.importorskip("scipy")
pytest.importorskip("numpy")

from native_physical_run import PROFILE_PATH, ROOT, SCHEMAS, run_three_physical_chunks  # noqa: E402
from robot_backend_sim.motion_config import load_control_config  # noqa: E402
from robot_contracts.native import validate_accepted_reference  # noqa: E402
from robot_contracts.profiles import load_profile  # noqa: E402


def test_three_physical_chunks_have_real_references_retirement_and_measurements():
    run = run_three_physical_chunks()
    profile = load_profile(PROFILE_PATH, SCHEMAS)
    control = load_control_config(ROOT / "simulation/mujoco/configs/control/panda_control.yaml")
    assert len(run.references) == 3
    assert [len(record.request.trajectory.targets) for record in run.references] == [60, 60, 60]
    assert [len(record.selected_indices) for record in run.references] == [20, 20, 18]
    for record in run.references:
        validate_accepted_reference(record, profile, control.max_accelerations)
    retired = {event.chunk_seq for event in run.events if event.phase == "RETIRED"}
    assert retired == {1, 2, 3}
    overall = [event for event in run.events if event.chunk_seq is None]
    assert [event.phase for event in overall].count("COMPLETED") == 1
    assert not any(
        event.phase == "COMPLETED" and event.chunk_seq is not None for event in run.events
    )
    assert run.terminal.phase == "COMPLETED"
    assert run.terminal.finished_sim_ns is not None
    assert all(capture.sim_time_ns >= 0 for capture in run.captures)
    # At least one real measured position differs from its reference; the test
    # never copies reference values into the measurement field.
    assert any(
        any(
            abs(reference - measured) > 1e-9
            for reference, measured in zip(
                execution.reference_positions, execution.measured_positions, strict=True
            )
        )
        for execution in run.feedback
        if execution.reference_positions
    )
