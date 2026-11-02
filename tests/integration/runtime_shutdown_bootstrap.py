"""Test-only runtime bootstrap: inject finish/executor faults, then call the real main.

This file lives under tests so the production runtime has no test hooks. The ROS node,
signals, executor, recorder and subprocess exit path remain the real implementations.
"""

import os
import threading
import time
from pathlib import Path

from dual_arm_runtime import control_client as control_client_module
from dual_arm_runtime import episode as episode_module
from dual_arm_runtime import node as node_module
from dual_arm_runtime import recording as recording_module
from dual_arm_runtime import run_logging as run_logging_module
from robot_run_logging import EventLogger, LogFailure


def _positive_float(name: str) -> float:
    raw = os.environ.get(name, "")
    try:
        value = float(raw)
    except ValueError:
        return 0.0
    return value if value > 0 else 0.0


def main() -> None:
    hold = _positive_float("DSH003_TEST_FINISH_HOLD_S")
    if hold:
        marker = os.environ.get("DSH003_TEST_FINISH_MARKER", "")
        original_audit = recording_module.audit_recording

        def delayed_audit(*args, **kwargs):
            if marker:
                Path(marker).write_text("finish-audit-entered\n")
            time.sleep(hold)
            return original_audit(*args, **kwargs)

        recording_module.audit_recording = delayed_audit

    fail_spin_after = os.environ.get("DSH003_TEST_FAIL_SPIN_AFTER", "")
    if fail_spin_after:
        limit = int(fail_spin_after)
        original_spin = node_module.SingleThreadedExecutor.spin_once
        calls = 0

        def failing_spin(self, *args, **kwargs):
            nonlocal calls
            calls += 1
            if calls > limit:
                raise RuntimeError("injected executor failure")
            return original_spin(self, *args, **kwargs)

        node_module.SingleThreadedExecutor.spin_once = failing_spin

    shutdown_spin_marker = os.environ.get("DSH003_TEST_FAIL_SHUTDOWN_SPIN", "")
    if shutdown_spin_marker:
        from rclpy.executors import ExternalShutdownException

        original_request = node_module.RuntimeNode.request_shutdown
        original_spin = node_module.SingleThreadedExecutor.spin_once
        shutdown_started = False

        def mark_shutdown(self, reason):
            nonlocal shutdown_started
            original_request(self, reason)
            shutdown_started = True

        def fail_shutdown_spin(self, *args, **kwargs):
            if shutdown_started:
                with Path(shutdown_spin_marker).open("a") as stream:
                    stream.write("failed-spin\n")
                raise ExternalShutdownException()
            return original_spin(self, *args, **kwargs)

        node_module.RuntimeNode.request_shutdown = mark_shutdown
        node_module.SingleThreadedExecutor.spin_once = fail_shutdown_spin

    fail_status_phase = os.environ.get("DSH003_TEST_FAIL_STATUS_PHASE", "")
    if fail_status_phase:
        original_status = episode_module.EpisodeRunner._set_status

        def failing_status(self, phase, *args, **kwargs):
            if phase == fail_status_phase:
                raise RuntimeError(f"injected {phase} status failure")
            return original_status(self, phase, *args, **kwargs)

        episode_module.EpisodeRunner._set_status = failing_status

    if os.environ.get("DSH003_TEST_FAIL_FINISH"):

        def failing_finish(self, *args, **kwargs):
            raise RuntimeError("injected finish failure")

        recording_module.EpisodeRecorder.finish = failing_finish

    fail_write_event = os.environ.get("DSH008_TEST_FAIL_WRITE_EVENT", "")
    if fail_write_event:
        original_write_item = EventLogger._write_item
        needle = fail_write_event.encode()

        def failing_write_item(self, item):
            if needle in item.body:
                raise OSError(f"injected async write failure for {fail_write_event}")
            return original_write_item(self, item)

        EventLogger._write_item = failing_write_item

    fail_emit_event = os.environ.get("DSH008_TEST_FAIL_EMIT_EVENT", "")
    if fail_emit_event:
        original_emit = EventLogger.emit
        fail_after = max(1, int(os.environ.get("DSH008_TEST_FAIL_EMIT_AFTER", "1")))
        seen = 0

        def failing_emit(self, event, *args, **kwargs):
            nonlocal seen
            if event == fail_emit_event:
                seen += 1
                if seen >= fail_after:
                    raise LogFailure(f"injected critical emit failure for {event}")
            return original_emit(self, event, *args, **kwargs)

        EventLogger.emit = failing_emit

    if os.environ.get("DSH008_TEST_CLOSE_ERROR"):
        original_close = EventLogger.close

        def failing_close(self):
            original_close(self)
            raise LogFailure("injected close failure")

        EventLogger.close = failing_close

    if os.environ.get("DSH008_TEST_METADATA_ERROR"):

        def failing_metadata(path, document):
            raise OSError("injected metadata failure")

        run_logging_module._write_json_atomic = failing_metadata

    command_marker = os.environ.get("DSH008_TEST_COMMAND_MARKER", "")
    if command_marker:
        original_execute = control_client_module.ControlClient.execute

        def marked_execute(self, command_message):
            with Path(command_marker).open("a") as stream:
                stream.write(f"{command_message.command_id}\n")
            return original_execute(self, command_message)

        control_client_module.ControlClient.execute = marked_execute

    fail_control_call = os.environ.get("DSH008_TEST_FAIL_CONTROL_CALL", "")
    if fail_control_call:
        original_control_call = control_client_module.ControlClient._call

        def failing_control_call(self, name, request, **kwargs):
            if name == fail_control_call:
                raise RuntimeError(f"injected {name} failure")
            return original_control_call(self, name, request, **kwargs)

        control_client_module.ControlClient._call = failing_control_call

    control_call_marker = os.environ.get("DSH008_TEST_CONTROL_CALL_MARKER", "")
    if control_call_marker:
        original_call = control_client_module.ControlClient._call

        def marked_call(self, name, request, **kwargs):
            with Path(control_call_marker).open("a") as stream:
                stream.write(f"{name}\n")
            return original_call(self, name, request, **kwargs)

        control_client_module.ControlClient._call = marked_call

    cleanup_marker = os.environ.get("DSH003_TEST_CLEANUP_MARKER", "")
    if cleanup_marker:
        original_cleanup = control_client_module.ControlClient.cleanup

        def marked_cleanup(self):
            try:
                return original_cleanup(self)
            finally:
                Path(cleanup_marker).write_text("cleanup-ran\n")

        control_client_module.ControlClient.cleanup = marked_cleanup

    slow_write_topic = os.environ.get("DSH009_TEST_SLOW_WRITE_TOPIC", "")
    if slow_write_topic:
        hold = _positive_float("DSH009_TEST_SLOW_WRITE_HOLD_S")
        original_write = recording_module._NativeWriter.write
        held = []

        def slow_write(self, topic, payload, timestamp_ns):
            if topic == slow_write_topic and not held:
                held.append(True)
                time.sleep(hold)
            return original_write(self, topic, payload, timestamp_ns)

        recording_module._NativeWriter.write = slow_write

    fail_write_topic = os.environ.get("DSH009_TEST_FAIL_WRITE_TOPIC", "")
    if fail_write_topic:
        original_write = recording_module._NativeWriter.write

        def failing_write(self, topic, payload, timestamp_ns):
            if topic == fail_write_topic:
                raise OSError(f"injected writer failure for {topic}")
            return original_write(self, topic, payload, timestamp_ns)

        recording_module._NativeWriter.write = failing_write

    close_hold = _positive_float("DSH009_TEST_CLOSE_HOLD_S")
    if close_hold:
        original_close = recording_module._NativeWriter.close

        def held_close(self):
            time.sleep(close_hold)
            return original_close(self)

        recording_module._NativeWriter.close = held_close

    terminal_delay = _positive_float("DSH010_TEST_DELAY_TERMINAL_S")
    drop_terminal = bool(os.environ.get("DSH010_TEST_DROP_TERMINAL"))
    if terminal_delay or drop_terminal:
        original_event = node_module.RuntimeNode._on_event

        def delayed_terminal(self, message):
            if message.phase == "CANCELED":
                if drop_terminal:
                    return
                threading.Timer(
                    terminal_delay,
                    lambda: original_event(self, message),
                ).start()
                return
            return original_event(self, message)

        node_module.RuntimeNode._on_event = delayed_terminal

    node_module.main()


if __name__ == "__main__":
    main()
