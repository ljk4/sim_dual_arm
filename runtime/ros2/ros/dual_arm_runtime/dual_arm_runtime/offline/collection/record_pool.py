"""One campaign owner schedules concurrent, isolated recording processes."""

from __future__ import annotations

import argparse
import concurrent.futures
import json
import os
import signal
import subprocess
import sys
import time
from dataclasses import dataclass
from functools import partial
from pathlib import Path
from typing import Any, TextIO

from robot_contracts.project_paths import project_path

from dual_arm_runtime.offline.collection import campaign
from dual_arm_runtime.offline.collection.campaign_execution import (
    _environment,
    acquire_run_lock,
    attempt_log_directory,
    handle_sigterm,
    shutil_disk_free,
    stop_process,
)
from dual_arm_runtime.offline.collection.campaign_ledger import (
    Attempt,
    CampaignError,
    Ledger,
    attempts,
    check_environment,
    finished,
)
from dual_arm_runtime.offline.collection.collect_attempt import check_ros_sdk


@dataclass
class ActiveRecording:
    attempt: Attempt
    worker: dict[str, Any]
    process: subprocess.Popen
    console: TextIO
    started: float


def load_worker_plan(path: Path) -> tuple[dict[str, Any], list[dict[str, Any]]]:
    document = json.loads(path.read_text(encoding="utf-8"))
    workers = document.get("workers", [])
    if not 1 <= len(workers) <= 16:
        raise CampaignError("worker manifest must contain 1..16 workers")
    for field in ("id", "domain_id", "namespace", "runtime_config", "backend_log"):
        if len({item[field] for item in workers}) != len(workers):
            raise CampaignError(f"workers must have distinct {field}")
    selection = document.get("selection")
    if selection is not None and selection.get("mode") == "scenario":
        binding = selection.get("binding") or {}
        if not binding.get("sha256") or not selection.get("scenario_id"):
            raise CampaignError("scenario worker manifest lacks binding identity")
        recipes = selection.get("recipes") or {}
        capability = (selection.get("capabilities") or {}).get("teacher")
        if not capability:
            raise CampaignError("scenario worker manifest lacks a Teacher capability")
        for worker in workers:
            if worker.get("teacher_api") != capability:
                raise CampaignError("workers must share the selected Teacher capability")
            worker_binding = worker.get("binding") or {}
            if worker_binding.get("sha256") != binding["sha256"] or worker_binding.get(
                "path"
            ) != binding.get("path"):
                raise CampaignError("workers must share the selected binding identity")
            for field in ("runtime_teacher", "simulation_teacher"):
                worker_recipe = (worker.get("recipes") or {}).get(field) or {}
                selected_recipe = recipes.get(field) or {}
                if worker_recipe.get("path") != selected_recipe.get("path") or worker_recipe.get(
                    "sha256"
                ) != selected_recipe.get("sha256"):
                    raise CampaignError(f"workers must consume the selected {field} recipe")
    if selection is not None:
        for field in ("run_id_prefix", "output_root"):
            if any(not worker.get(field) for worker in workers):
                raise CampaignError(f"workers must declare {field}")
            if len({worker[field] for worker in workers}) != len(workers):
                raise CampaignError(f"workers must have distinct {field}")
    for worker in workers:
        domain = worker["domain_id"]
        if isinstance(domain, bool) or not isinstance(domain, int) or not 0 <= domain <= 232:
            raise CampaignError("worker ROS domain IDs must be integers in 0..232")
        if not Path(worker["runtime_config"]).is_file():
            raise CampaignError(f"worker runtime config is missing: {worker['runtime_config']}")
        if not Path(worker["backend_log"]).is_dir():
            raise CampaignError(f"worker backend is not ready: {worker['backend_log']}")
    return document, workers


class RecordingPool:
    def __init__(
        self,
        args: argparse.Namespace,
        config: dict[str, Any],
        ledger: Ledger,
        workers: list[dict[str, Any]],
        selection: dict[str, Any] | None = None,
    ) -> None:
        self.args, self.config, self.ledger, self.workers = args, config, ledger, workers
        self.selection = selection
        self.output = project_path(config["paths"]["attempts_root"], args.project_root)
        self.stop_grace_s = float(config["launch"].get("stop_grace_s", 10.0))
        self.active: list[ActiveRecording] = []
        self.qualified = self.attempted = self.cancelled = 0
        self.worker_failures: dict[str, int] = {
            worker["id"]: 0 for worker in workers
        }
        self.started = time.monotonic()
        self._last_progress_render = 0.0
        self._run_id_counter: dict[str, int] = {}
        self.reason, self.exit_code = "target_reached", 0
        self.allocation_error = ""

    @property
    def consecutive_failures(self) -> int:
        """Largest current failure streak across the independent worker processes."""
        return max(self.worker_failures.values(), default=0)

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
        from dual_arm_runtime.offline.collection.record import _collector_command

        run_args = argparse.Namespace(
            purpose=self.args.purpose,
            kind="rollout",
            mode="teacher",
            seed=None,
            episode_id=None,
            config=self.args.config,
            runtime_config=worker["runtime_config"],
        )
        check_environment(self.args.project_root, self.config, self.output)
        run_id_prefix = worker.get("run_id_prefix") or f"record-{worker['id']}"
        base_run_id = f"{run_id_prefix}_{time.strftime('%Y%m%d_%H%M%S')}"
        count = self._run_id_counter.get(base_run_id, 0) + 1
        self._run_id_counter[base_run_id] = count
        attempt_run_id = base_run_id if count == 1 else f"{base_run_id}_{count}"
        attempt = campaign._reserve(
            self.ledger,
            self.config,
            run_args,
            self.output,
            attempt_run_id,
            self.args.project_root,
        )
        self.attempted += 1
        with self.ledger.locked():
            self.ledger.append(
                {
                    "event": "RUNNING",
                    "attempt_id": attempt.attempt_id,
                    "started_at_ns": time.time_ns(),
                    "worker": worker["id"],
                    "domain_id": worker["domain_id"],
                    "run_id_prefix": run_id_prefix,
                    "output_root": worker.get("output_root"),
                    "binding": worker.get("binding"),
                    "recipes": worker.get("recipes"),
                    "teacher_api": worker.get("teacher_api"),
                    "worker_overrides": worker.get("overrides"),
                }
            )
        collect_args = argparse.Namespace(
            **{
                **vars(self.args),
                "runtime_config": Path(worker["runtime_config"]),
                "backend_log": Path(worker["backend_log"]),
                "robot_namespace": worker["namespace"],
            }
        )
        environment = _environment(attempt)
        environment.update(
            ROS_DOMAIN_ID=str(worker["domain_id"]),
            OMP_NUM_THREADS="1",
            OPENBLAS_NUM_THREADS="1",
            MKL_NUM_THREADS="1",
            CAMPAIGN_WORKER_ID=worker["id"],
            CAMPAIGN_WORKER_RUN_ID_PREFIX=run_id_prefix,
        )
        if worker.get("output_root"):
            environment["CAMPAIGN_WORKER_OUTPUT_ROOT"] = worker["output_root"]
        binding = worker.get("binding") or {}
        if binding:
            environment["CAMPAIGN_SCENARIO_ID"] = str(binding.get("scenario_id") or "")
            environment["CAMPAIGN_ROBOT"] = str(binding.get("robot") or "")
            environment["CAMPAIGN_TASK"] = str(binding.get("task") or "")
            environment["CAMPAIGN_BINDING_PATH"] = str(binding.get("path") or "")
            environment["CAMPAIGN_BINDING_SHA256"] = str(binding.get("sha256") or "")
        if worker.get("teacher_api"):
            environment["CAMPAIGN_TEACHER_API"] = str(worker["teacher_api"])
        for name in ("DISPLAY", "WAYLAND_DISPLAY"):
            environment.pop(name, None)
        log_directory = attempt_log_directory(self.config, self.args.project_root, attempt)
        log_directory.mkdir(parents=True, exist_ok=True)
        console = (log_directory / "console.log").open("w", encoding="utf-8")
        try:
            process = subprocess.Popen(
                _collector_command(collect_args),
                cwd=self.args.project_root,
                env=environment,
                start_new_session=True,
                stdout=console,
                stderr=subprocess.STDOUT,
            )
        except OSError as error:
            console.close()
            _, result, _ = campaign._finish_attempt(
                self.ledger, attempt, self.config, 127, 0.0, str(error)
            )
            self.completed(result, worker["id"])
            return
        self.active.append(ActiveRecording(attempt, worker, process, console, time.monotonic()))
        self._render_progress()

    def _is_tty(self) -> bool:
        return sys.stdout.isatty() or os.environ.get("RECORD_PROGRESS_TTY") == "1"

    def _format_duration(self, seconds: float | None) -> str:
        if seconds is None:
            return "--:--:--"
        total = max(0, round(seconds))
        minutes, secs = divmod(total, 60)
        hours, minutes = divmod(minutes, 60)
        return f"{hours:02d}:{minutes:02d}:{secs:02d}"

    def _estimated_remaining(self, elapsed: float) -> float | None:
        if self.qualified >= self.args.count:
            return 0.0
        if self.qualified <= 0:
            return None
        return elapsed * (self.args.count - self.qualified) / self.qualified

    def _render_progress(self, *, force: bool = False) -> None:
        is_tty = self._is_tty()
        if not is_tty and not force:
            return
        total = max(1, self.args.count)
        ratio = min(1.0, self.qualified / total)
        width = 20
        filled = int(width * ratio)
        bar = "█" * filled + "░" * (width - filled)
        elapsed = time.monotonic() - self.started
        eta = self._estimated_remaining(elapsed)
        line = (
            f"并行采集 [{bar}] {self.qualified}/{self.args.count} "
            f"| 在途 {len(self.active)} "
            f"| 最大连败 {self.consecutive_failures}/{self.args.max_consecutive_failures} "
            f"| 用时 {self._format_duration(elapsed)} "
            f"| 剩余 {self._format_duration(eta)}"
        )
        self._last_progress_render = time.monotonic()
        if is_tty:
            sys.stdout.write(f"\r\033[K{line}")
            sys.stdout.flush()
        else:
            print(line, flush=True)

    def completed(self, result: dict[str, Any], worker_id: str) -> None:
        if result["qualified"]:
            self.qualified += 1
            self.worker_failures[worker_id] = 0
        else:
            self.worker_failures[worker_id] = self.worker_failures.get(worker_id, 0) + 1
        self._render_progress()
        if self.consecutive_failures >= self.args.max_consecutive_failures:
            self.reason, self.exit_code = "consecutive_failures", 1

    def finish(self, active: ActiveRecording, code: int, failure: str = "") -> None:
        self.active.remove(active)
        active.console.close()
        _, result, _ = campaign._finish_attempt(
            self.ledger,
            active.attempt,
            self.config,
            code,
            time.monotonic() - active.started,
            failure,
        )
        self.completed(result, active.worker["id"])

    def cancel_active(self) -> None:
        if self.active:
            stop = partial(stop_process, stop_grace_s=self.stop_grace_s)
            with concurrent.futures.ThreadPoolExecutor(max_workers=len(self.active)) as executor:
                list(executor.map(stop, [item.process for item in self.active]))
        for active in tuple(self.active):
            active.console.close()
            campaign._finish_attempt(
                self.ledger,
                active.attempt,
                self.config,
                130,
                time.monotonic() - active.started,
                f"batch stopped: {self.reason}",
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
                        stop_process(active.process, self.stop_grace_s)
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
                    if self._is_tty() and time.monotonic() - self._last_progress_render >= 0.5:
                        self._render_progress()
                    time.sleep(0.05)
        except KeyboardInterrupt:
            self.reason, self.exit_code = "interrupted", 130
        finally:
            self.cancel_active()
        self._render_progress(force=not self._is_tty())
        if self._is_tty():
            sys.stdout.write("\n")
            sys.stdout.flush()
        elapsed = time.monotonic() - self.started
        summary = {
            "event": "recording.finished",
            "reason": self.reason,
            "requested": self.args.count,
            "qualified": self.qualified,
            "attempted": self.attempted,
            "cancelled": self.cancelled,
            "consecutive_failures": self.consecutive_failures,
            "max_consecutive_failures": self.args.max_consecutive_failures,
            "workers": len(self.workers),
            "elapsed_s": round(elapsed, 3),
            "qualified_per_hour": round(self.qualified * 3600 / max(elapsed, 0.001), 1),
            "error": self.allocation_error or None,
            "target_total": self.args.target_total,
            "campaign_qualified": self.campaign_qualified(),
            "selection": self.selection,
        }
        if self.args.purpose == "formal" and self.reason == "target_reached":
            with self.ledger.locked():
                records = self.ledger.records()
            destination = campaign.write_formal_selection(
                self.args.project_root, self.config, records, self.args.config
            )
            if destination is not None:
                summary["formal_select"] = str(destination)
        self.args.worker_manifest.with_name("summary.json").write_text(
            json.dumps(summary, indent=2) + "\n", encoding="utf-8"
        )
        reason_text = {
            "target_reached": "目标达成",
            "consecutive_failures": "最大连败达到上限",
            "interrupted": "已中断",
            "disk_full": "磁盘空间不足",
            "campaign_stopped": "采集停止",
        }.get(self.reason, self.reason)
        print(
            f"并行采集结束：{reason_text} | 合格 {self.qualified}/{self.args.count} | "
            f"已尝试 {self.attempted} | 取消 {self.cancelled} | "
            f"用时 {self._format_duration(elapsed)} | "
            f"合格速率 {summary['qualified_per_hour']}/小时",
            flush=True,
        )
        if "formal_select" in summary:
            print(f"合格录制清单: {summary['formal_select']}", flush=True)
        print(f"运行摘要: {self.args.worker_manifest.with_name('summary.json')}", flush=True)
        return self.exit_code


def record_parallel(args: argparse.Namespace, config: dict[str, Any]) -> int:
    document, workers = load_worker_plan(args.worker_manifest)
    selection = document.get("selection")
    expected = args.selection
    if expected is not None:
        if selection is None:
            raise CampaignError("worker manifest has no selection but --robot/--task was used")
        if expected.get("binding", {}).get("sha256") != selection.get("binding", {}).get("sha256"):
            raise CampaignError("worker manifest binding differs from the record selection")
    code = check_ros_sdk(
        args.project_root,
        robot=args.robot,
        task=args.task,
        shared_root=args.shared_root,
    )
    if code:
        return code
    paths = config["paths"]
    ledger = Ledger(
        project_path(paths["ledger"], args.project_root),
        project_path(paths["lock"], args.project_root),
    )
    run_lock = project_path(paths["run_lock"], args.project_root)
    acquire_run_lock(run_lock)
    previous = signal.signal(signal.SIGTERM, handle_sigterm)
    try:
        pool = RecordingPool(args, config, ledger, workers, selection)
        if args.target_total is not None:
            if args.purpose == "formal" and args.target_total > int(
                config["budget"]["formal_target_qualified"]
            ):
                raise CampaignError("target total exceeds the campaign formal qualified target")
            args.count = max(0, args.target_total - pool.campaign_qualified())
        return pool.run()
    finally:
        signal.signal(signal.SIGTERM, previous)
        run_lock.unlink(missing_ok=True)
