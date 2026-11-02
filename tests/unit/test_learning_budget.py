import json
import time
from pathlib import Path

import pytest
from dual_arm_learning.budget import BudgetError, BudgetExceeded, BudgetLedger


def ledger(tmp_path: Path, limit: float = 100.0) -> BudgetLedger:
    return BudgetLedger(tmp_path / "budget.jsonl", tmp_path / "budget.lock", limit)


def begin(budget: BudgetLedger, segment: str, reserved: float = 1.0):
    return budget.begin(
        run_id="stable-run",
        segment_id=segment,
        mode="smoke",
        reserved_seconds=reserved,
        config_sha256="a" * 64,
        instance_id=segment,
    )


def test_same_stable_run_two_segments_are_cumulative(tmp_path):
    budget = ledger(tmp_path)
    first = begin(budget, "segment-1", 10.0)
    first.finish("completed")
    second = begin(budget, "segment-2", 20.0)
    second.finish("completed")
    status = budget.status()
    segments = status["runs"]["stable-run"]["segments"]
    # The stable run is not overwritten by the second segment; charges add.
    assert len(segments) == 2
    assert status["runs"]["stable-run"]["charged_seconds"] == pytest.approx(
        sum(segment["charged_seconds"] for segment in segments.values())
    )
    assert status["records"] == 6


def test_duplicate_segment_and_concurrent_begin_are_rejected(tmp_path):
    budget = ledger(tmp_path)
    first = begin(budget, "segment-1")
    with pytest.raises(BudgetError, match="lifecycle lock"):
        begin(budget, "segment-2")
    first.finish("completed")
    with pytest.raises(BudgetError, match="already exists"):
        begin(budget, "segment-1")


def test_reservation_beyond_limit_is_rejected_without_writing(tmp_path):
    budget = ledger(tmp_path, limit=10.0)
    with pytest.raises(BudgetExceeded):
        begin(budget, "too-big", 11.0)
    assert not budget.path.exists() or budget.path.read_text() == ""


def test_active_status_does_not_freeze_or_recover_live_segment(tmp_path):
    budget = ledger(tmp_path)
    ticket = begin(budget, "live", 5.0)
    status = budget.status()
    assert status["active"] is True
    assert status["runs"]["stable-run"]["segments"]["live"]["active"] is True
    records = [json.loads(line) for line in budget.path.read_text().splitlines() if line]
    assert not any(record["event"] == "ORPHAN_CHARGED" for record in records)
    ticket.finish("completed")
    assert budget.status()["active"] is False


def test_crashed_segment_is_charged_once_and_never_reset(tmp_path):
    budget = ledger(tmp_path, limit=100.0)
    now = time.time_ns()
    budget.path.write_text(
        "\n".join(
            json.dumps(record)
            for record in (
                {
                    "event": "RESERVED",
                    "run_id": "stable-run",
                    "segment_id": "crashed",
                    "mode": "overfit",
                    "reserved_seconds": 9.0,
                    "wall_time_ns": now - 50_000_000_000,
                    "config_sha256": "c" * 64,
                },
                {
                    "event": "STARTED",
                    "run_id": "stable-run",
                    "segment_id": "crashed",
                    "wall_time_ns": now - 40_000_000_000,
                    "monotonic_ns": 0,
                },
            )
        )
        + "\n",
        encoding="utf-8",
    )
    first = budget.status()
    assert first["runs"]["stable-run"]["segments"]["crashed"]["orphan"] is True
    assert first["runs"]["stable-run"]["charged_seconds"] >= 40.0
    second = budget.status()
    assert (
        second["runs"]["stable-run"]["charged_seconds"]
        == first["runs"]["stable-run"]["charged_seconds"]
    )
    records = [json.loads(line) for line in budget.path.read_text().splitlines() if line]
    assert sum(record["event"] == "ORPHAN_CHARGED" for record in records) == 1


def test_counterexample_10s_plus_20s_same_run_is_at_least_30s(tmp_path):
    budget = ledger(tmp_path, limit=100.0)
    now = time.time_ns()
    lines = []
    for segment, elapsed in (("first", 10.0), ("second", 20.0)):
        lines.append(
            {
                "event": "RESERVED",
                "run_id": "resumed-run",
                "segment_id": segment,
                "mode": "overfit",
                "reserved_seconds": elapsed,
                "wall_time_ns": now - int(elapsed * 1e9),
                "config_sha256": "d" * 64,
            }
        )
        lines.append(
            {
                "event": "FINISHED",
                "run_id": "resumed-run",
                "segment_id": segment,
                "mode": "overfit",
                "outcome": "completed",
                "elapsed_seconds": elapsed,
                "reserved_seconds": elapsed,
            }
        )
    budget.path.write_text("\n".join(json.dumps(line) for line in lines) + "\n")
    status = budget.status()
    assert status["runs"]["resumed-run"]["charged_seconds"] == pytest.approx(30.0)


def test_cross_process_lifecycle_lock_and_active_status(tmp_path):
    import os
    import subprocess
    import sys

    budget_path = tmp_path / "budget.jsonl"
    lock_path = tmp_path / "budget.lock"
    ready = tmp_path / "ready"
    script = f"""
from pathlib import Path
import time
from dual_arm_learning.budget import BudgetLedger
b=BudgetLedger({str(budget_path)!r}, {str(lock_path)!r}, 100.0)
t=b.begin(run_id='stable-run', segment_id='child', mode='smoke',
    reserved_seconds=30.0, config_sha256='e'*64)
Path({str(ready)!r}).write_text('ready')
time.sleep(20)
"""
    env = dict(os.environ)
    env["PYTHONDONTWRITEBYTECODE"] = "1"
    child = subprocess.Popen([sys.executable, "-c", script], env=env)
    try:
        deadline = time.monotonic() + 10
        while not ready.is_file() and time.monotonic() < deadline:
            time.sleep(0.02)
        assert ready.is_file()
        budget = BudgetLedger(budget_path, lock_path, 100.0)
        status = budget.status()
        assert status["active"] is True
        assert status["runs"]["stable-run"]["segments"]["child"]["active"] is True
        with pytest.raises(BudgetError, match="lifecycle lock"):
            budget.begin(
                run_id="stable-run",
                segment_id="parent",
                mode="smoke",
                reserved_seconds=1.0,
                config_sha256="f" * 64,
            )
    finally:
        child.kill()
        child.wait(timeout=10)
    status = BudgetLedger(budget_path, lock_path, 100.0).status()
    assert status["runs"]["stable-run"]["segments"]["child"]["orphan"] is True
    assert status["runs"]["stable-run"]["segments"]["child"]["active"] is False
