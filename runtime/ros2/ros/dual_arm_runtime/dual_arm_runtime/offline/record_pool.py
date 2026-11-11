"""One campaign owner schedules concurrent, isolated recording processes."""

from __future__ import annotations

import argparse
import concurrent.futures
import json
import os
import signal
import subprocess
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any, TextIO

from dual_arm_runtime.offline import campaign
from dual_arm_runtime.offline.campaign_execution import (
    _environment,
    acquire_run_lock,
    handle_sigterm,
    shutil_disk_free,
)
from dual_arm_runtime.offline.campaign_ledger import (
    Attempt,
    CampaignError,
    Ledger,
    attempts,
    check_environment,
    finished,
)
from dual_arm_runtime.offline.collect_attempt import check_ros_sdk


@dataclass
class ActiveRecording:
    attempt: Attempt
    worker: dict[str, Any]
    process: subprocess.Popen
    console: TextIO
    started: float


def load_workers(path: Path) -> list[dict[str, Any]]:
    document = json.loads(path.read_text())
    workers = document.get("workers", [])
    if document.get("schema_version") != 1 or not 1 <= len(workers) <= 16:
        raise CampaignError("worker manifest must contain 1..16 workers with schema_version 1")
    for field in ("id", "domain_id", "namespace", "runtime_config", "backend_log"):
        if len({item[field] for item in workers}) != len(workers):
            raise CampaignError(f"workers must have distinct {field}")
    for worker in workers:
        domain = worker["domain_id"]
        if isinstance(domain, bool) or not isinstance(domain, int) or not 0 <= domain <= 232:
            raise CampaignError("worker ROS domain IDs must be integers in 0..232")
        if not Path(worker["runtime_config"]).is_file():
            raise CampaignError(f"worker runtime config is missing: {worker['runtime_config']}")
        if not Path(worker["backend_log"]).is_dir():
            raise CampaignError(f"worker backend is not ready: {worker['backend_log']}")
    return workers


def stop_process(process: subprocess.Popen) -> None:
    try:
        os.killpg(process.pid, signal.SIGTERM)
    except ProcessLookupError:
        process.wait()
        return
    deadline = time.monotonic() + 10
    while time.monotonic() < deadline:
        process.poll()
        try:
            os.killpg(process.pid, 0)
        except ProcessLookupError:
            break
        time.sleep(0.05)
    else:
        try:
            os.killpg(process.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass
    process.wait(timeout=5)


class RecordingPool:
    def __init__(
        self, args: argparse.Namespace, config: dict[str, Any], ledger: Ledger,
        workers: list[dict[str, Any]],
    ) -> None:
        self.args, self.config, self.ledger, self.workers = args, config, ledger, workers
        self.output = args.project_root / config["paths"]["attempts_root"]
        self.active: list[ActiveRecording] = []
        self.qualified = self.attempted = self.consecutive_failures = self.cancelled = 0
        self.started = time.monotonic()
        self.reason, self.exit_code = "target_reached", 0
        self.allocation_error = ""

    def campaign_qualified(self) -> int:
        with self.ledger.locked():
            records = self.ledger.records()
        terminal = finished(records)
        return sum(
            bool(terminal.get(item["attempt_id"], {}).get("qualified"))
            for item in attempts(records, self.args.purpose)
        )

    def available_slots(self) -> int:
        remaining = self.args.count - self.qualified
        if self.args.purpose == "formal":
            remaining = min(
                remaining,
                int(self.config["budget"]["formal_target_qualified"]) - self.campaign_qualified(),
            )
        return max(0, remaining - len(self.active))

    def launch(self, worker: dict[str, Any]) -> None:
        from dual_arm_runtime.offline.record import _collector_command

        run_args = argparse.Namespace(
            purpose=self.args.purpose, kind="rollout", mode="teacher", seed=None,
            episode_id=None, runtime_config=worker["runtime_config"],
        )
        check_environment(self.args.project_root, self.config, self.output)
        attempt = campaign._reserve(
            self.ledger, self.config, run_args, self.output,
            f"record-{time.time_ns()}-{worker['id']}", self.args.project_root,
        )
        self.attempted += 1
        with self.ledger.locked():
            self.ledger.append(
                {
                    "event": "RUNNING", "attempt_id": attempt.attempt_id,
                    "started_at_ns": time.time_ns(), "worker": worker["id"],
                    "domain_id": worker["domain_id"],
                }
            )
        collect_args = argparse.Namespace(
            **{**vars(self.args), "backend_log": Path(worker["backend_log"]), "no_previews": True}
        )
        environment = _environment(attempt)
        environment.update(
            ROS_DOMAIN_ID=str(worker["domain_id"]), MUJOCO_GL="egl",
            OMP_NUM_THREADS="1", OPENBLAS_NUM_THREADS="1", MKL_NUM_THREADS="1",
        )
        for name in ("DISPLAY", "WAYLAND_DISPLAY"):
            environment.pop(name, None)
        console = (attempt.directory / "console.log").open("w")
        try:
            process = subprocess.Popen(
                _collector_command(collect_args), cwd=self.args.project_root,
                env=environment, start_new_session=True, stdout=console, stderr=subprocess.STDOUT,
            )
        except OSError as error:
            console.close()
            _, result, _ = campaign._finish_attempt(
                self.ledger, attempt, self.config, 127, 0.0, str(error)
            )
            self.completed(result)
            return
        self.active.append(ActiveRecording(attempt, worker, process, console, time.monotonic()))
        print(json.dumps({"event": "recording.started", "seed": attempt.seed,
                          "worker": worker["id"], "attempt_id": attempt.attempt_id}), flush=True)

    def completed(self, result: dict[str, Any]) -> None:
        if result["qualified"]:
            self.qualified += 1
            self.consecutive_failures = 0
        else:
            self.consecutive_failures += 1
        print(
            f"并行录制：合格 {self.qualified}/{self.args.count}，已尝试 {self.attempted} 次，"
            f"连续失败 {self.consecutive_failures}/{self.args.max_consecutive_failures}",
            flush=True,
        )
        if self.consecutive_failures >= self.args.max_consecutive_failures:
            self.reason, self.exit_code = "consecutive_failures", 1

    def finish(self, active: ActiveRecording, code: int, failure: str = "") -> None:
        self.active.remove(active)
        active.console.close()
        event, result, raw_bytes = campaign._finish_attempt(
            self.ledger, active.attempt, self.config, code,
            time.monotonic() - active.started, failure,
        )
        print(json.dumps({"event": event, "attempt_id": active.attempt.attempt_id,
                          "worker": active.worker["id"], "raw_bytes": raw_bytes,
                          **result}, sort_keys=True), flush=True)
        self.completed(result)

    def cancel_active(self) -> None:
        if self.active:
            with concurrent.futures.ThreadPoolExecutor(max_workers=len(self.active)) as executor:
                list(executor.map(stop_process, [item.process for item in self.active]))
        for active in tuple(self.active):
            active.console.close()
            campaign._finish_attempt(
                self.ledger, active.attempt, self.config, 130,
                time.monotonic() - active.started, f"batch stopped: {self.reason}",
            )
            self.active.remove(active)
            self.cancelled += 1

    def run(self) -> int:
        try:
            while self.qualified < self.args.count and self.exit_code == 0:
                for active in tuple(self.active):
                    code = active.process.poll()
                    if code is None and time.monotonic() - active.started >= float(
                        self.config["launch"].get("timeout_s", 180)
                    ):
                        stop_process(active.process)
                        code = 124
                    if code is not None:
                        self.finish(active, code, "attempt timed out" if code == 124 else "")
                        if code == 130:
                            self.reason, self.exit_code = "interrupted", 130
                        if self.exit_code:
                            break
                if self.exit_code:
                    break
                if shutil_disk_free(self.args.project_root) < int(
                    self.config["budget"]["min_free_bytes"]
                ):
                    self.reason, self.exit_code = "disk_full", 2
                    break
                busy = {item.worker["id"] for item in self.active}
                before_launch = self.attempted
                for worker in self.workers:
                    if worker["id"] in busy or self.available_slots() == 0 or self.allocation_error:
                        continue
                    try:
                        self.launch(worker)
                    except CampaignError as error:
                        self.allocation_error = str(error)
                    if self.exit_code:
                        break
                if not self.active and self.qualified < self.args.count and self.exit_code == 0:
                    if self.allocation_error or self.attempted == before_launch:
                        self.reason, self.exit_code = "campaign_stopped", 2
                if self.active and self.exit_code == 0:
                    time.sleep(0.05)
        except KeyboardInterrupt:
            self.reason, self.exit_code = "interrupted", 130
        finally:
            self.cancel_active()
        elapsed = time.monotonic() - self.started
        summary = {
            "event": "recording.finished", "reason": self.reason,
            "requested": self.args.count, "qualified": self.qualified,
            "attempted": self.attempted, "cancelled": self.cancelled,
            "consecutive_failures": self.consecutive_failures,
            "max_consecutive_failures": self.args.max_consecutive_failures,
            "workers": len(self.workers), "elapsed_s": round(elapsed, 3),
            "qualified_per_hour": round(self.qualified * 3600 / max(elapsed, 0.001), 1),
            "error": self.allocation_error or None,
            "target_total": self.args.target_total,
            "campaign_qualified": self.campaign_qualified(),
        }
        self.args.worker_manifest.with_name("summary.json").write_text(
            json.dumps(summary, indent=2) + "\n"
        )
        print(json.dumps(summary, sort_keys=True), flush=True)
        return self.exit_code


def record_parallel(args: argparse.Namespace, config: dict[str, Any]) -> int:
    workers = load_workers(args.worker_manifest)
    code = check_ros_sdk(args.project_root)
    if code:
        return code
    paths = config["paths"]
    ledger = Ledger(args.project_root / paths["ledger"], args.project_root / paths["lock"])
    run_lock = args.project_root / paths["run_lock"]
    acquire_run_lock(run_lock)
    previous = signal.signal(signal.SIGTERM, handle_sigterm)
    try:
        pool = RecordingPool(args, config, ledger, workers)
        if args.target_total is not None:
            if args.purpose == "formal" and args.target_total > int(
                config["budget"]["formal_target_qualified"]
            ):
                raise CampaignError("target total exceeds the campaign formal qualified target")
            # Recheck after taking the campaign run lock: another batch may have
            # finished while the host was starting this batch's backends.
            args.count = max(0, args.target_total - pool.campaign_qualified())
        return pool.run()
    finally:
        signal.signal(signal.SIGTERM, previous)
        run_lock.unlink(missing_ok=True)
