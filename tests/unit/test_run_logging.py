"""Behavioural tests for the standalone structured event logger."""

from __future__ import annotations

import json
import math
import threading
import time
from pathlib import Path

import jsonschema
import pytest
from robot_run_logging import EventContext, EventLogger, LogFailure, LogSettings

ROOT = Path(__file__).resolve().parents[2]
SCHEMA = json.loads(
    (ROOT / "contracts/schemas/v1/run_event.schema.json").read_text(encoding="utf-8")
)
VALIDATOR = jsonschema.Draft202012Validator(SCHEMA)


def make_logger(tmp_path: Path, **overrides: object) -> EventLogger:
    settings = LogSettings(**overrides)
    return EventLogger(
        tmp_path / "run" / "component" / "instance",
        run_id="run-1",
        component="test",
        settings=settings,
    )


def event_files(logger: EventLogger) -> list[Path]:
    return sorted(logger.directory.glob("events.*.jsonl"))


def read_events(logger: EventLogger) -> list[tuple[dict[str, object], int]]:
    events: list[tuple[dict[str, object], int]] = []
    for path in event_files(logger):
        for line in path.read_bytes().splitlines(keepends=True):
            events.append((json.loads(line), len(line)))
    return events


def read_summary(logger: EventLogger) -> dict[str, object]:
    return json.loads((logger.directory / "summary.json").read_text(encoding="utf-8"))


def wait_for_failure(logger: EventLogger, timeout: float = 2.0) -> None:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        try:
            logger.check_health()
        except LogFailure:
            return
        time.sleep(0.005)
    raise AssertionError("logger did not report the expected failure")


def test_event_shape_context_schema_and_summary(tmp_path: Path) -> None:
    logger = make_logger(tmp_path)
    assert read_summary(logger)["state"] == "running"
    assert logger.emit(
        "run.started",
        message="started",
        reason="startup",
        context=EventContext(
            episode_id="episode-1",
            request_id="request-1",
            command_id="command-1",
            clock_epoch=3,
            sim_time_ns=42,
        ),
        fields={"count": 2, "ratio": 0.5, "enabled": True, "note": "ok", "empty": None},
        exception=ValueError("boom"),
    )
    assert logger.emit("diagnostic", level="DEBUG", critical=False)
    logger.close()
    logger.close()

    events = read_events(logger)
    assert [event["event_seq"] for event, _ in events] == [1, 2]
    for event, _ in events:
        VALIDATOR.validate(event)
    first = events[0][0]
    assert first["schema_version"] == 1
    assert first["run_id"] == "run-1"
    assert first["component"] == "test"
    assert first["instance_id"] == logger.instance_id
    assert first["event"] == "run.started"
    assert first["reason"] == "startup"
    assert first["message"] == "started"
    assert first["critical"] is True
    assert first["episode_id"] == "episode-1"
    assert first["request_id"] == "request-1"
    assert first["command_id"] == "command-1"
    assert first["clock_epoch"] == 3
    assert first["sim_time_ns"] == 42
    assert first["fields"] == {
        "count": 2,
        "ratio": 0.5,
        "enabled": True,
        "note": "ok",
        "empty": None,
    }
    assert first["exception"]["type"] == "ValueError"
    assert first["exception"]["message"] == "boom"
    assert first["exception"]["truncated"] is False
    assert "ValueError: boom" in first["exception"]["stacktrace"]
    second = events[1][0]
    assert second["episode_id"] is None
    assert second["clock_epoch"] is None
    assert second["fields"] == {}
    assert second["exception"] is None

    summary = read_summary(logger)
    assert summary["state"] == "complete"
    assert summary["reason"] is None
    assert summary["attempted"] == 2
    assert summary["enqueued"] == 2
    assert summary["written"] == 2
    assert summary["dropped_noncritical"] == 0
    assert summary["files"] == [path.name for path in event_files(logger)]


def test_directory_is_exclusive_and_instance_defaults(tmp_path: Path) -> None:
    logger = make_logger(tmp_path)
    assert logger.directory.is_dir()
    assert logger.instance_id
    with pytest.raises(FileExistsError):
        EventLogger(
            logger.directory,
            run_id="run-2",
            component="test",
            settings=LogSettings(),
        )
    logger.close()
    with pytest.raises(LogFailure):
        logger.emit("too.late")


def test_concurrent_producers_have_one_sequence_order(tmp_path: Path) -> None:
    logger = make_logger(
        tmp_path,
        queue_capacity=2048,
        max_event_bytes=4096,
        max_file_bytes=4 * 1024 * 1024,
        max_total_bytes=16 * 1024 * 1024,
        close_timeout_s=5.0,
        flush_interval_s=0.01,
    )
    barrier = threading.Barrier(9)
    accepted: list[bool] = []
    accepted_lock = threading.Lock()

    def producer(worker: int) -> None:
        barrier.wait()
        for index in range(50):
            ok = logger.emit(
                f"worker.{worker}",
                critical=False,
                fields={"worker": worker, "index": index},
            )
            with accepted_lock:
                accepted.append(ok)

    threads = [threading.Thread(target=producer, args=(worker,)) for worker in range(8)]
    for thread in threads:
        thread.start()
    barrier.wait()
    for thread in threads:
        thread.join()
    logger.close()

    assert all(accepted)
    events = read_events(logger)
    assert [event["event_seq"] for event, _ in events] == list(range(1, 401))
    assert len({event["fields"]["worker"] for event, _ in events}) == 8
    summary = read_summary(logger)
    assert summary["written"] == 400
    assert summary["dropped_noncritical"] == 0


def test_rotation_preserves_every_event_in_order(tmp_path: Path) -> None:
    logger = make_logger(
        tmp_path,
        max_event_bytes=1024,
        max_file_bytes=2048,
        max_total_bytes=1024 * 1024,
        flush_interval_s=0.01,
    )
    for index in range(20):
        assert logger.emit(
            "rotation.event",
            critical=False,
            message="x" * 200,
            fields={"index": index},
        )
    logger.close()

    paths = event_files(logger)
    assert len(paths) > 1
    events = read_events(logger)
    assert [event["event_seq"] for event, _ in events] == list(range(1, 21))
    assert [event["fields"]["index"] for event, _ in events] == list(range(20))
    assert all(line_size <= 1024 for _, line_size in events)
    summary = read_summary(logger)
    assert summary["state"] == "complete"
    assert summary["written"] == 20
    assert summary["files"] == [path.name for path in paths]


def test_total_budget_fails_without_overshooting(tmp_path: Path) -> None:
    logger = make_logger(
        tmp_path,
        max_event_bytes=1024,
        max_file_bytes=2048,
        max_total_bytes=2048,
        close_timeout_s=2.0,
        flush_interval_s=0.01,
    )
    for _ in range(10):
        try:
            logger.emit("budget.event", critical=False, message="y" * 100)
        except LogFailure:
            break
    wait_for_failure(logger)
    with pytest.raises(LogFailure):
        logger.close()

    total_event_bytes = sum(path.stat().st_size for path in event_files(logger))
    assert total_event_bytes <= 2048
    summary = read_summary(logger)
    assert summary["state"] == "failed"
    assert summary["reason"]
    assert summary["attempted"] == summary["enqueued"] + summary["dropped_noncritical"]
    assert summary["written"] <= summary["enqueued"]


def test_noncritical_queue_full_is_counted(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    logger = make_logger(
        tmp_path,
        queue_capacity=1,
        close_timeout_s=2.0,
        flush_interval_s=0.01,
    )
    from robot_run_logging import writer as writer_module

    entered = threading.Event()
    release = threading.Event()
    original_sync = writer_module._sync_file

    def blocking_sync(fd: int) -> None:
        entered.set()
        assert release.wait(5)
        original_sync(fd)

    monkeypatch.setattr(writer_module, "_sync_file", blocking_sync)
    assert logger.emit("blocker")
    assert entered.wait(2)
    assert logger.emit("queued", critical=False)
    assert logger.emit("dropped.one", critical=False) is False
    assert logger.emit("dropped.two", critical=False) is False
    logger.check_health()
    release.set()
    logger.close()

    summary = read_summary(logger)
    assert summary["state"] == "complete"
    assert summary["attempted"] == 4
    assert summary["enqueued"] == 2
    assert summary["written"] == 2
    assert summary["dropped_noncritical"] == 2
    assert [event["event_seq"] for event, _ in read_events(logger)] == [1, 2]


def test_critical_queue_full_sets_permanent_failure(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    logger = make_logger(
        tmp_path,
        queue_capacity=1,
        close_timeout_s=2.0,
        flush_interval_s=0.01,
    )
    from robot_run_logging import writer as writer_module

    entered = threading.Event()
    release = threading.Event()
    original_sync = writer_module._sync_file

    def blocking_sync(fd: int) -> None:
        entered.set()
        assert release.wait(5)
        original_sync(fd)

    monkeypatch.setattr(writer_module, "_sync_file", blocking_sync)
    assert logger.emit("blocker")
    assert entered.wait(2)
    assert logger.emit("queued", critical=False)
    with pytest.raises(LogFailure):
        logger.emit("critical.lost")
    with pytest.raises(LogFailure):
        logger.check_health()
    release.set()
    with pytest.raises(LogFailure):
        logger.close()
    with pytest.raises(LogFailure):
        logger.close()

    summary = read_summary(logger)
    assert summary["state"] == "failed"
    assert "queue full" in summary["reason"]
    assert summary["written"] == 2
    assert [event["event_seq"] for event, _ in read_events(logger)] == [1, 2]


def test_sync_failure_is_visible_and_summary_is_failed(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    logger = make_logger(tmp_path, close_timeout_s=2.0, flush_interval_s=0.01)
    from robot_run_logging import writer as writer_module

    original_sync = writer_module._sync_file
    calls = 0

    def failing_sync(fd: int) -> None:
        nonlocal calls
        calls += 1
        if calls == 1:
            raise OSError("injected fsync failure")
        original_sync(fd)

    monkeypatch.setattr(writer_module, "_sync_file", failing_sync)
    assert logger.emit("will.fail")
    with pytest.raises(LogFailure):
        logger.close()
    with pytest.raises(LogFailure):
        logger.check_health()

    summary = read_summary(logger)
    assert summary["state"] == "failed"
    assert "fsync failure" in summary["reason"]
    assert summary["written"] == 1


def test_rotation_open_failure_is_visible(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    logger = make_logger(
        tmp_path,
        max_event_bytes=1024,
        max_file_bytes=1024,
        max_total_bytes=1024 * 1024,
        close_timeout_s=2.0,
        flush_interval_s=0.01,
    )
    from robot_run_logging import writer as writer_module

    original_open = open

    def failing_open(file: object, mode: str = "r", *args: object, **kwargs: object):
        if "events.000002.jsonl" in str(file):
            raise OSError("injected rotation open failure")
        return original_open(file, mode, *args, **kwargs)

    monkeypatch.setattr(writer_module, "open", failing_open, raising=False)
    for _ in range(6):
        try:
            logger.emit("rotate.event", critical=False, message="z" * 100)
        except LogFailure:
            break
    with pytest.raises(LogFailure):
        logger.close()
    summary = read_summary(logger)
    assert summary["state"] == "failed"
    assert "rotation open failure" in summary["reason"]


def test_close_timeout_is_bounded_and_sticky(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    logger = make_logger(
        tmp_path,
        close_timeout_s=0.1,
        flush_interval_s=0.01,
    )
    from robot_run_logging import writer as writer_module

    entered = threading.Event()
    release = threading.Event()
    original_sync = writer_module._sync_file

    def blocking_sync(fd: int) -> None:
        entered.set()
        assert release.wait(5)
        original_sync(fd)

    monkeypatch.setattr(writer_module, "_sync_file", blocking_sync)
    assert logger.emit("blocked.event")
    assert entered.wait(2)
    started = time.monotonic()
    with pytest.raises(LogFailure):
        logger.close()
    elapsed = time.monotonic() - started
    assert 0.05 <= elapsed < 1.0
    with pytest.raises(LogFailure):
        logger.check_health()
    with pytest.raises(LogFailure):
        logger.close()
    release.set()
    assert logger._writer_done.wait(2)  # type: ignore[attr-defined]

    summary = read_summary(logger)
    assert summary["state"] == "failed"
    assert "timed out" in summary["reason"]


def test_long_message_and_exception_are_truncated_with_markers(tmp_path: Path) -> None:
    logger = make_logger(
        tmp_path,
        max_event_bytes=2048,
        max_file_bytes=4096,
        max_total_bytes=1024 * 1024,
    )
    assert logger.emit(
        "long.event",
        message="m" * 20000,
        fields={"payload": "value"},
        exception=ValueError("e" * 10000),
    )
    logger.close()

    events = read_events(logger)
    assert len(events) == 1
    event, line_size = events[0]
    assert line_size <= 2048
    VALIDATOR.validate(event)
    assert "truncated" in event["message"]
    assert "original_chars=20000" in event["message"]
    assert event["exception"]["truncated"] is True
    assert event["exception"]["message_original_chars"] == 10000
    assert int(event["exception"]["stacktrace_original_chars"]) > 10000
    assert "truncated" in event["exception"]["message"]


@pytest.mark.parametrize(
    "kwargs",
    [
        {"message": 1},
        {"event": "has space"},
        {"level": "info"},
        {"reason": "has space"},
        {"fields": {"bad key": 1}},
        {"fields": {"nan": float("nan")}},
        {"fields": {"inf": math.inf}},
        {"fields": {"list": [1, 2]}},
        {"fields": {"object": object()}},
        {"fields": {"too_long": "x" * 2049}},
        {"fields": {f"f{index}": index for index in range(65)}},
        {"context": EventContext(episode_id="bad id")},
        {"context": EventContext(clock_epoch=True)},
        {"exception": "not an exception"},
    ],
)
def test_invalid_payloads_fail_at_emit(tmp_path: Path, kwargs: dict[str, object]) -> None:
    logger = make_logger(tmp_path)
    options = dict(kwargs)
    event = options.pop("event", "valid.event")
    with pytest.raises(ValueError):
        logger.emit(event, **options)
    logger.close()
    assert read_events(logger) == []
    summary = read_summary(logger)
    assert summary["attempted"] == 0


def test_event_larger_than_max_event_bytes_is_rejected(tmp_path: Path) -> None:
    logger = make_logger(
        tmp_path,
        max_event_bytes=128,
        max_file_bytes=256,
        max_total_bytes=1024,
    )
    with pytest.raises(ValueError):
        logger.emit("oversized.event", message="x" * 1000)
    logger.close()
    assert read_events(logger) == []
    assert read_summary(logger)["state"] == "complete"


def test_settings_validation(tmp_path: Path) -> None:
    assert LogSettings().queue_capacity == 256
    assert LogSettings().max_event_bytes == 32768
    with pytest.raises(ValueError):
        LogSettings(queue_capacity=0)
    with pytest.raises(ValueError):
        LogSettings(queue_capacity=True)
    with pytest.raises(ValueError):
        LogSettings(close_timeout_s=math.nan)
    with pytest.raises(ValueError):
        LogSettings(flush_interval_s=0)
    with pytest.raises(ValueError):
        LogSettings(max_event_bytes=20, max_file_bytes=10, max_total_bytes=100)
    with pytest.raises(ValueError):
        LogSettings(max_file_bytes=20, max_total_bytes=10)
    existing = tmp_path / "existing"
    existing.mkdir()
    with pytest.raises(FileExistsError):
        EventLogger(
            existing,
            run_id="run",
            component="test",
            settings=LogSettings(),
        )


def test_critical_overflow_reports_on_writer_thread(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    logger = make_logger(
        tmp_path,
        queue_capacity=1,
        close_timeout_s=2.0,
        flush_interval_s=0.01,
    )
    from robot_run_logging import writer as writer_module

    sync_entered = threading.Event()
    sync_release = threading.Event()
    stderr_entered = threading.Event()
    stderr_release = threading.Event()
    original_sync = writer_module._sync_file

    def blocking_sync(fd: int) -> None:
        sync_entered.set()
        assert sync_release.wait(5)
        original_sync(fd)

    class SlowStderr:
        def write(self, text: str) -> int:
            if text.startswith("run logging failure:"):
                stderr_entered.set()
                assert stderr_release.wait(5)
            return len(text)

        def flush(self) -> None:
            pass

    monkeypatch.setattr(writer_module, "_sync_file", blocking_sync)
    monkeypatch.setattr(writer_module.sys, "stderr", SlowStderr())
    try:
        assert logger.emit("prime")
        assert sync_entered.wait(2)
        assert logger.emit("queued")
        started = time.monotonic()
        with pytest.raises(LogFailure):
            logger.emit("overflow")
        assert time.monotonic() - started < 0.2
        assert stderr_entered.is_set() is False
        sync_release.set()
        assert stderr_entered.wait(2)
        stderr_release.set()
        with pytest.raises(LogFailure):
            logger.close()
    finally:
        sync_release.set()
        stderr_release.set()
        try:
            logger.close()
        except LogFailure:
            pass


def test_late_complete_summary_commit_cannot_erase_timeout(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    logger = make_logger(tmp_path, close_timeout_s=0.1, flush_interval_s=0.02)
    from robot_run_logging import writer as writer_module

    rename_entered = threading.Event()
    rename_release = threading.Event()
    committed = threading.Event()
    original_replace = writer_module.os.replace

    def slow_replace(source: object, destination: object) -> None:
        if (
            Path(destination) == logger.directory / "summary.json"
            and json.loads(Path(source).read_text(encoding="utf-8"))["state"] == "complete"
        ):
            rename_entered.set()
            assert rename_release.wait(5)
            original_replace(source, destination)
            committed.set()
            return
        original_replace(source, destination)

    monkeypatch.setattr(writer_module.os, "replace", slow_replace)
    assert logger.emit("before.close")
    try:
        started = time.monotonic()
        with pytest.raises(LogFailure):
            logger.close()
        assert time.monotonic() - started < 1.0
        assert rename_entered.wait(2)
    finally:
        rename_release.set()
    assert committed.wait(2)
    deadline = time.monotonic() + 2.0
    while time.monotonic() < deadline:
        if read_summary(logger)["state"] == "failed":
            break
        time.sleep(0.01)
    assert read_summary(logger)["state"] == "failed"
    with pytest.raises(LogFailure):
        logger.close()


def test_emit_after_writer_failure_rejects_noncritical(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    logger = make_logger(tmp_path, close_timeout_s=2.0, flush_interval_s=0.01)
    from robot_run_logging import writer as writer_module

    original_sync = writer_module._sync_file
    calls = 0

    def failing_sync(fd: int) -> None:
        nonlocal calls
        calls += 1
        if calls == 1:
            raise OSError("injected fsync failure")
        original_sync(fd)

    monkeypatch.setattr(writer_module, "_sync_file", failing_sync)
    assert logger.emit("will.fail")
    assert logger._writer_done.wait(2)  # type: ignore[attr-defined]
    with pytest.raises(LogFailure):
        logger.emit("after.exit", critical=False)
    with pytest.raises(LogFailure):
        logger.close()


def test_concurrent_close_calls_each_finish_within_budget(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    logger = make_logger(tmp_path, close_timeout_s=0.1, flush_interval_s=0.01)
    from robot_run_logging import writer as writer_module

    sync_entered = threading.Event()
    sync_release = threading.Event()
    original_sync = writer_module._sync_file

    def blocking_sync(fd: int) -> None:
        sync_entered.set()
        assert sync_release.wait(5)
        original_sync(fd)

    monkeypatch.setattr(writer_module, "_sync_file", blocking_sync)
    logger.emit("blocked.event")
    assert sync_entered.wait(2)
    outcomes: list[tuple[float, str | None]] = []

    def close() -> None:
        started = time.monotonic()
        reason = None
        try:
            logger.close()
        except LogFailure as exception:
            reason = str(exception)
        outcomes.append((time.monotonic() - started, reason))

    closers = [threading.Thread(target=close) for _ in range(2)]
    try:
        for thread in closers:
            thread.start()
        for thread in closers:
            thread.join(0.4)
        assert all(not thread.is_alive() for thread in closers)
        assert len(outcomes) == 2
        assert all(elapsed < 0.3 and reason is not None for elapsed, reason in outcomes)
    finally:
        sync_release.set()
        for thread in closers:
            thread.join(2)
    assert logger._writer_done.wait(2)
    assert read_summary(logger)["state"] == "failed"
    with pytest.raises(LogFailure):
        logger.close()


def test_close_does_not_wait_for_suspended_producer(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    from robot_run_logging.events import _PreparedEvent

    logger = make_logger(tmp_path, close_timeout_s=0.1)
    render_entered = threading.Event()
    render_release = threading.Event()
    close_done = threading.Event()
    original_render = _PreparedEvent.render
    producer_errors: list[Exception] = []
    close_errors: list[Exception] = []

    def slow_render(self: object, **kwargs: object) -> object:
        render_entered.set()
        assert render_release.wait(5)
        return original_render(self, **kwargs)

    def produce() -> None:
        try:
            logger.emit("suspended.event")
        except Exception as exception:
            producer_errors.append(exception)

    def close() -> None:
        try:
            logger.close()
        except Exception as exception:
            close_errors.append(exception)
        finally:
            close_done.set()

    monkeypatch.setattr(_PreparedEvent, "render", slow_render)
    producer = threading.Thread(target=produce)
    closer = threading.Thread(target=close)
    producer.start()
    assert render_entered.wait(2)
    try:
        closer.start()
        assert close_done.wait(0.3), "close waited for event rendering"
        assert close_errors == []
    finally:
        render_release.set()
        producer.join(2)
        closer.join(2)
    assert len(producer_errors) == 1
    assert isinstance(producer_errors[0], LogFailure)
    assert read_events(logger) == []
    assert read_summary(logger)["state"] == "complete"
    assert read_summary(logger)["enqueued"] == 0
