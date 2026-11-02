"""Human runtime CLI behavior against the real RuntimeNode and public services."""

from __future__ import annotations

import json
import os
import time
from types import SimpleNamespace

import pytest
from dual_arm_runtime import cli
from runtime_cli_harness import (
    RuntimeCliHarness,
    client_domain,
    run_cli,
    wait_active,
    wait_ready,
)


@pytest.fixture
def harness(tmp_path):
    value = RuntimeCliHarness(tmp_path)
    value.start_backend()
    value.start_runtime()
    try:
        wait_ready(value)
        yield value
    finally:
        value.stop()


def test_main_status_returns_zero_and_exactly_one_json(harness, capsys):
    with client_domain(harness):
        code = cli.main(
            ["--run-id", harness.run_id, "--timeout", "10", "status"],
            raw_root=harness.raw_root,
        )

    assert code == 0
    lines = [line for line in capsys.readouterr().out.splitlines() if line.strip()]
    assert len(lines) == 1
    result = json.loads(lines[0])
    assert result["success"] is True
    assert result["status"]["phase"] == "IDLE"


def test_initial_idle_status_and_discoverable_services(harness):
    result = run_cli(harness, "status", 10)

    assert result["success"] is True
    assert result["status"]["phase"] == "IDLE"
    assert result["status"]["episode_id"] == ""
    assert result["services"] == {"start": True, "end": True}
    assert result["healthy"] is True
    assert result["audit_verified"] is False


def test_start_wait_completes_then_a_second_episode_ignores_the_old_idle(harness):
    first = run_cli(harness, "start", 60, task_id="mock-script", wait=True)

    assert first["success"] is True, first
    assert first["reason"] == "COMPLETED"
    assert first["ack"] == {
        "success": True,
        "reason": "PREPARING",
        "episode_id": first["episode_id"],
    }
    assert first["manifest"]["recording"]["complete"] is True
    assert first["manifest"]["runtime"]["outcome"] == "completed"
    assert first["manifest"]["task"]["outcome"] == "not_evaluated"
    assert first["manifest"]["audit_verified"] is False

    idle = run_cli(harness, "status", 10)
    assert idle["status"]["phase"] == "IDLE"
    assert idle["status"]["episode_id"] == first["episode_id"]

    second = run_cli(harness, "start", 60, task_id="mock-script", wait=True)
    assert second["success"] is True, second
    assert second["episode_id"] != first["episode_id"]
    assert second["manifest"]["runtime"]["outcome"] == "completed"

    idle_again = run_cli(harness, "status", 10)
    assert idle_again["status"]["episode_id"] == second["episode_id"]


def test_end_wait_stops_the_named_episode_and_reads_its_manifest(harness):
    started = run_cli(harness, "start", 30, task_id="mock-script")
    assert started["success"] is True
    episode_id = started["episode_id"]
    wait_active(harness, episode_id)

    ended = run_cli(
        harness,
        "end",
        60,
        episode_id=episode_id,
        reason="OPERATOR_STOP",
        wait=True,
    )

    assert ended["success"] is True, ended
    assert ended["reason"] == "STOPPED"
    assert ended["ack"] == {"success": True, "reason": "ENDING"}
    assert ended["status"]["phase"] == "IDLE"
    assert ended["status"]["episode_id"] == episode_id
    assert ended["manifest"]["recording"] == {"complete": True, "failure_reason": ""}
    assert ended["manifest"]["runtime"]["outcome"] == "stopped"
    assert ended["manifest"]["runtime"]["reason"] == "OPERATOR_STOP"
    assert ended["manifest"]["task"]["outcome"] == "not_evaluated"
    assert ended["manifest"]["audit_verified"] is False


def test_wrong_episode_id_is_not_sent_and_the_correct_end_still_works(harness):
    started = run_cli(harness, "start", 30, task_id="mock-script")
    episode_id = started["episode_id"]
    wait_active(harness, episode_id)

    wrong = run_cli(harness, "end", 5, episode_id="episode-missing")
    assert wrong["success"] is False
    assert wrong["reason"] == "EPISODE_MISMATCH"
    assert "ack" not in wrong
    current = run_cli(harness, "status", 10)
    assert current["status"]["episode_id"] == episode_id

    correct = run_cli(harness, "end", 60, episode_id=episode_id, reason="OPERATOR_STOP", wait=True)
    assert correct["success"] is True, correct
    assert correct["manifest"]["runtime"]["outcome"] == "stopped"


def test_second_start_while_active_is_rejected(harness):
    started = run_cli(harness, "start", 30, task_id="mock-script")
    episode_id = started["episode_id"]
    wait_active(harness, episode_id)

    repeated = run_cli(harness, "start", 10, task_id="mock-script")

    assert repeated["success"] is False
    assert repeated["reason"] == "START_REJECTED"
    assert repeated["ack"]["success"] is False
    assert repeated["ack"]["reason"] == "CONTROL_BUSY"

    ended = run_cli(harness, "end", 60, episode_id=episode_id, reason="OPERATOR_STOP", wait=True)
    assert ended["success"] is True, ended


def test_wait_timeout_does_not_accept_the_previous_episode_terminal(harness):
    first = run_cli(harness, "start", 60, task_id="mock-script", wait=True)
    assert first["success"] is True

    second = run_cli(harness, "start", 1.5, task_id="mock-script", wait=True)

    assert second["success"] is False
    assert second["reason"] in {"WAIT_TIMEOUT", "EPISODE_MISMATCH"}
    assert second["episode_id"] != first["episode_id"]
    assert "manifest" not in second

    # Clean up only if the timed-out episode is still active.  It may have
    # completed while the test was observing the timeout.
    episode_id = second["episode_id"]
    for _ in range(20):
        current = run_cli(harness, "status", 10)["status"]
        if current["episode_id"] == episode_id and current["phase"] == "ACTIVE":
            ended = run_cli(
                harness,
                "end",
                60,
                episode_id=episode_id,
                reason="OPERATOR_STOP",
                wait=True,
            )
            if not ended["success"]:
                # The timed motion can finish between the status read and End RPC.
                assert ended["reason"] == "EPISODE_NOT_ACTIVE", ended
                terminal = run_cli(harness, "status", 10)["status"]
                assert terminal["episode_id"] == episode_id
                assert terminal["phase"] == "IDLE"
            break
        if current["episode_id"] == episode_id and current["phase"] in {"IDLE", "FAULT"}:
            break
        time.sleep(0.05)


def test_status_is_not_healthy_after_the_runtime_process_is_lost(tmp_path):
    value = RuntimeCliHarness(tmp_path)
    value.start_backend()
    value.start_runtime()
    try:
        wait_ready(value)
        value.stop_runtime()
        result = run_cli(value, "status", 3)
        assert result["success"] is False
        assert result["reason"] in {
            "SERVICE_UNAVAILABLE",
            "STATUS_UNAVAILABLE",
            "EPISODE_MISMATCH",
        }
        assert result.get("healthy") is not True
    finally:
        value.stop()


def test_manifest_reader_rejects_a_symlink_that_escapes_the_raw_root(tmp_path):
    raw_root = tmp_path / "raw"
    run_directory = raw_root / "manual-cli-test"
    run_directory.mkdir(parents=True)
    outside = tmp_path / "outside"
    outside.mkdir()
    (outside / "manifest.json").write_text(
        json.dumps(
            {
                "schema_version": 2,
                "run_id": "manual-cli-test",
                "episode_id": "episode-evil",
                "recording": {"complete": True, "failure_reason": ""},
                "runtime": {"outcome": "completed", "reason": ""},
                "task": {"outcome": "not_evaluated", "reason": ""},
            }
        )
    )
    (run_directory / "episode-evil").symlink_to(outside, target_is_directory=True)

    with pytest.raises(cli.CliError) as caught:
        cli.read_manifest(raw_root, "manual-cli-test", "episode-evil")

    assert caught.value.reason in {"MANIFEST_OUTSIDE_ROOT", "MANIFEST_INVALID"}


def test_fault_cannot_be_upgraded_by_a_complete_completed_manifest():
    manifest = {
        "schema_version": 2,
        "run_id": "manual-cli-test",
        "episode_id": "episode-fault",
        "recording": {"complete": True, "failure_reason": ""},
        "runtime": {"outcome": "completed", "reason": ""},
        "task": {"outcome": "not_evaluated", "reason": ""},
        "audit_verified": False,
    }
    client = SimpleNamespace(read_manifest=lambda run_id, episode_id: manifest)
    args = SimpleNamespace(run_id="manual-cli-test")
    terminal = cli.StatusSnapshot("FAULT", "episode-fault", "RECORDING_ERROR: injected", "")
    ack = cli.Ack(True, "PREPARING", "episode-fault")

    with pytest.raises(cli.CliError) as caught:
        cli._manifest_result(
            client,
            args,
            time.monotonic() + 1,
            cli.RuntimeOutcome.COMPLETED,
            episode_id="episode-fault",
            terminal=terminal,
            ack=ack,
        )

    assert caught.value.reason == "RUNTIME_FAULT"
    assert caught.value.fields["manifest"] == manifest
    assert caught.value.fields["status"]["phase"] == "FAULT"


def test_cli_timeout_parser_rejects_non_finite_and_non_positive_values():
    for value in (True, False, 0, -1, float("nan"), float("inf"), "nan"):
        with pytest.raises(ValueError):
            cli.positive_timeout(value)
    assert cli.positive_timeout("2.5") == 2.5


def test_manifest_reader_rejects_oversize_bad_json_and_wrong_identity(tmp_path):
    episode = tmp_path / "raw" / "manual-cli-test" / "episode-1"
    episode.mkdir(parents=True)
    manifest = episode / "manifest.json"

    # Characters under the cap, bytes over it: the limit must be byte-bounded.
    manifest.write_text("é" * (cli.MAX_MANIFEST_BYTES // 2 + 1), encoding="utf-8")
    with pytest.raises(cli.CliError) as too_large:
        cli.read_manifest(tmp_path / "raw", "manual-cli-test", "episode-1")
    assert too_large.value.reason == "MANIFEST_TOO_LARGE"

    manifest.write_text("{not json", encoding="utf-8")
    with pytest.raises(cli.CliError) as bad_json:
        cli.read_manifest(tmp_path / "raw", "manual-cli-test", "episode-1")
    assert bad_json.value.reason == "MANIFEST_INVALID"

    manifest.write_text(
        json.dumps(
            {
                "schema_version": 2,
                "run_id": "other-run",
                "episode_id": "episode-1",
                "recording": {"complete": True, "failure_reason": ""},
                "runtime": {"outcome": "completed", "reason": ""},
                "task": {"outcome": "not_evaluated", "reason": ""},
            }
        ),
        encoding="utf-8",
    )
    with pytest.raises(cli.CliError) as wrong_identity:
        cli.read_manifest(tmp_path / "raw", "manual-cli-test", "episode-1")
    assert wrong_identity.value.reason == "MANIFEST_INVALID"


def test_manifest_reader_rejects_a_fifo_without_blocking(tmp_path):
    episode = tmp_path / "raw" / "manual-cli-test" / "episode-1"
    episode.mkdir(parents=True)
    os.mkfifo(episode / "manifest.json")

    started = time.monotonic()
    with pytest.raises(cli.CliError) as caught:
        cli.read_manifest(tmp_path / "raw", "manual-cli-test", "episode-1")

    assert caught.value.reason == "MANIFEST_INVALID"
    assert time.monotonic() - started < 2


def test_manifest_reader_returns_dimensions_and_rejects_missing_files(tmp_path):
    with pytest.raises(cli.CliError) as missing:
        cli.read_manifest(tmp_path / "raw", "manual-cli-test", "episode-missing")
    assert missing.value.reason == "MANIFEST_MISSING"

    episode = tmp_path / "raw" / "manual-cli-test" / "episode-1"
    episode.mkdir(parents=True)
    (episode / "manifest.json").write_text(
        json.dumps(
            {
                "schema_version": 2,
                "run_id": "manual-cli-test",
                "episode_id": "episode-1",
                "recording": {"complete": False, "failure_reason": "CLOSE_TIMEOUT"},
                "runtime": {"outcome": "failed", "reason": "RECORDING_ERROR: CLOSE_TIMEOUT"},
                "task": {"outcome": "not_evaluated", "reason": ""},
            }
        )
    )
    manifest = cli.read_manifest(tmp_path / "raw", "manual-cli-test", "episode-1")
    assert manifest["recording"]["complete"] is False
    assert manifest["runtime"]["outcome"] == "failed"
    assert manifest["task"]["outcome"] == "not_evaluated"
    assert manifest["audit_verified"] is False
