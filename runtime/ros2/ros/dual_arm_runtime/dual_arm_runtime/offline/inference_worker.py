"""Run isolated parallel policy episodes against prepared simulation backends."""

from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
import threading
import time
import traceback
from pathlib import Path
from typing import Any

ROOT = Path(os.environ.get("PROJECT_ROOT") or Path(__file__).resolve().parents[6])

_GRASP_KEYS = (
    "source_not_on_table",
    "not_on_table",
    "source_grasped",
    "airborne_handover_sequence",
    "left_hold",
    "right_hold",
    "hold_lost",
)
_PLACE_KEYS = (
    "source_on_target_height",
    "source_xy_inside_target",
    "source_target_vertical_contact",
    "block_contained",
    "box_bottom_support",
    "source_tilt",
    "target_tilt",
)
_RELEASE_KEYS = (
    "no_finger_contact",
    "right_gripper_opening",
    "right_tcp_clearance",
    "left_released",
    "right_released",
)


def _conditions(task: dict[str, Any]) -> dict[str, bool]:
    metrics = task.get("metrics")
    result: dict[str, bool] = {}
    if isinstance(metrics, dict):
        for item in metrics.get("conditions", []):
            if isinstance(item, dict) and isinstance(item.get("name"), str):
                result[item["name"]] = bool(item.get("passed"))
    return result


def _stage_state(task: dict[str, Any]) -> tuple[str, str, dict[str, bool]]:
    """Return (failure_stage, failure_reason, reached_milestones)."""
    reason = str(task.get("failure_reason") or task.get("reason") or "")
    if not task.get("state") and not reason:
        return (
            "执行异常",
            "未产出有效的 task_result.json",
            {"grasp": False, "place": False, "release": False},
        )
    conditions = _conditions(task)
    reached = {
        "grasp": not any(
            conditions.get(name) is False for name in _GRASP_KEYS if name in conditions
        ),
        "place": not any(
            conditions.get(name) is False for name in _PLACE_KEYS if name in conditions
        ),
        "release": not any(
            conditions.get(name) is False for name in _RELEASE_KEYS if name in conditions
        ),
    }
    if task.get("state") == "SUCCEEDED":
        return "", reason, {"grasp": True, "place": True, "release": True, "stable": True}
    if reason.startswith("DROPPED"):
        return "搬运/放置（掉落）", reason, reached
    if not reached["grasp"]:
        return "抓取/交接", reason, reached
    if not reached["place"]:
        return "搬运/放置", reason, reached
    if not reached["release"]:
        return "释放/撤离", reason, reached
    return "稳定确认", reason, reached


def _classification(recordings: Path, run_id: str, exit_code: int) -> dict[str, Any]:
    episode_dir = recordings / run_id
    task: dict[str, Any] = {}
    manifest: dict[str, Any] = {}
    try:
        task = json.loads((episode_dir / "task_result.json").read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        task = {}
    try:
        manifest = json.loads((episode_dir / "manifest.json").read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        manifest = {}
    task_section = task.get("task") if isinstance(task.get("task"), dict) else {}
    recording = manifest.get("recording") if isinstance(manifest.get("recording"), dict) else {}
    runtime = manifest.get("runtime") if isinstance(manifest.get("runtime"), dict) else {}
    succeeded = bool(
        exit_code == 0
        and task_section.get("state") == "SUCCEEDED"
        and isinstance(runtime, dict)
        and runtime.get("outcome") in {"stopped", "completed"}
        and recording.get("complete") is True
    )
    failure_stage, failure_reason, reached = _stage_state(task_section)
    return {
        "succeeded": succeeded,
        "failure_stage": "" if succeeded else failure_stage,
        "failure_reason": "" if succeeded else failure_reason,
        "milestones": reached,
        "task_outcome": task_section.get("state") or "",
        "runtime_outcome": str(runtime.get("outcome") or ""),
        "recording_complete": bool(recording.get("complete")),
        "record_directory": str(episode_dir),
    }


def _format_duration(seconds: float) -> str:
    total = max(0, int(seconds))
    minutes, secs = divmod(total, 60)
    hours, minutes = divmod(minutes, 60)
    return f"{hours:02d}:{minutes:02d}:{secs:02d}" if hours else f"{minutes:02d}:{secs:02d}"


class InferencePool:
    def __init__(self, args: argparse.Namespace, workers: list[dict[str, Any]]) -> None:
        self.args = args
        self.workers = workers
        self.recordings = args.recordings
        self.runs_root = args.runs_root
        self.lock = threading.Lock()
        self.worker_failures = {worker["id"]: 0 for worker in workers}
        self.episodes: list[dict[str, Any]] = []
        self.stopped = threading.Event()
        self.next_index = 0
        self.active = 0
        self.started = time.monotonic()

    @property
    def succeeded(self) -> int:
        return sum(1 for item in self.episodes if item["succeeded"])

    @property
    def failed(self) -> int:
        return len(self.episodes) - self.succeeded

    @property
    def max_failures(self) -> int:
        return max(self.worker_failures.values(), default=0)

    def _is_tty(self) -> bool:
        return sys.stdout.isatty()

    def _render_progress(self, *, force: bool = False) -> None:
        if not self._is_tty() and not force:
            return
        with self.lock:
            done = len(self.episodes)
            active = self.active
            succeeded = self.succeeded
            failures = self.max_failures
        total = max(1, self.args.count)
        fraction = min(1.0, done / total)
        filled = int(20 * fraction)
        bar = "█" * filled + "░" * (20 - filled)
        elapsed = _format_duration(time.monotonic() - self.started)
        limit = self.args.max_consecutive_failures or "-"
        line = (
            f"并行推理 [{bar}] {done}/{self.args.count} | 在途 {active} "
            f"| 成功 {succeeded} | 最大连败 {failures}/{limit} | {elapsed}"
        )
        if self._is_tty():
            sys.stdout.write(f"\r\033[K{line}")
            sys.stdout.flush()
        else:
            print(line, flush=True)

    def _take_index(self) -> int | None:
        with self.lock:
            if self.stopped.is_set() or self.next_index >= self.args.count:
                return None
            index = self.next_index
            self.next_index += 1
            return index

    def _episode_command(
        self, worker: dict[str, Any], run_id: str, seed: int
    ) -> list[str]:
        return [
            sys.executable,
            "-m",
            "dual_arm_runtime.runtime_node",
            "--project-root",
            str(ROOT),
            "--config",
            str(worker["runtime_config"]),
            "--once",
            "--run-id",
            run_id,
            "--episode-id",
            run_id,
            "--reset-seed",
            str(seed),
            "--output",
            str(self.recordings),
            "--runs-output",
            str(self.runs_root),
        ]

    def _run_episode(self, worker: dict[str, Any], index: int) -> None:
        seed = self.args.seed_start + index
        run_id = f"{self.args.batch_id}-{worker['id']}-ep{index:04d}"
        environment = dict(os.environ)
        environment.update(
            ROS_DOMAIN_ID=str(worker["domain_id"]),
            OMP_NUM_THREADS="1",
            OPENBLAS_NUM_THREADS="1",
            MKL_NUM_THREADS="1",
            DISPLAY="",
            WAYLAND_DISPLAY="",
        )
        console = self.args.console_root / f"{run_id}.log"
        console.parent.mkdir(parents=True, exist_ok=True)
        started = time.monotonic()
        with console.open("w", encoding="utf-8") as stream:
            result = subprocess.run(
                self._episode_command(worker, run_id, seed),
                cwd=ROOT,
                env=environment,
                stdout=stream,
                stderr=subprocess.STDOUT,
                check=False,
            )
        wall_s = time.monotonic() - started
        classified = _classification(self.recordings, run_id, result.returncode)
        record = {
            "index": index,
            "worker": worker["id"],
            "domain_id": worker["domain_id"],
            "seed": seed,
            "run_id": run_id,
            "exit_code": result.returncode,
            "wall_s": round(wall_s, 3),
            "console_log": str(console),
            **classified,
        }
        with self.lock:
            self.episodes.append(record)
            if record["succeeded"]:
                self.worker_failures[worker["id"]] = 0
            else:
                self.worker_failures[worker["id"]] += 1
            if (
                self.args.max_consecutive_failures is not None
                and self.worker_failures[worker["id"]]
                >= self.args.max_consecutive_failures
            ):
                self.stopped.set()
            completed = len(self.episodes)
        verdict = "成功" if record["succeeded"] else (
            f"失败 · {record['failure_stage']} · {record['failure_reason']}"
        )
        line = (
            f"[{completed}/{self.args.count}] {worker['id']} seed {seed} "
            f"{verdict} | {_format_duration(wall_s)} | {run_id}"
        )
        if self._is_tty():
            if not record["succeeded"]:
                sys.stdout.write(f"\r\033[2K{line}\n")
                sys.stdout.flush()
            self._render_progress()
        else:
            print(line, flush=True)

    def _worker_loop(self, worker: dict[str, Any]) -> None:
        while True:
            index = self._take_index()
            if index is None:
                return
            with self.lock:
                self.active += 1
            try:
                self._run_episode(worker, index)
            except Exception as error:  # noqa: BLE001 - one worker must not kill the batch
                with self.lock:
                    self.worker_failures[worker["id"]] += 1
                    self.episodes.append(
                        {
                            "index": index,
                            "worker": worker["id"],
                            "seed": self.args.seed_start + index,
                            "succeeded": False,
                            "failure_stage": "执行异常",
                            "failure_reason": f"{type(error).__name__}: {error}",
                            "traceback": traceback.format_exc(),
                        }
                    )
                    if (
                        self.args.max_consecutive_failures is not None
                        and self.worker_failures[worker["id"]]
                        >= self.args.max_consecutive_failures
                    ):
                        self.stopped.set()
                print(
                    f"{worker['id']} 执行异常：{type(error).__name__}: {error}",
                    file=sys.stderr,
                    flush=True,
                )
            finally:
                with self.lock:
                    self.active -= 1
                self._render_progress()

    def run(self) -> int:
        threads = [
            threading.Thread(target=self._worker_loop, args=(worker,), daemon=True)
            for worker in self.workers
        ]
        for thread in threads:
            thread.start()
        try:
            for thread in threads:
                while thread.is_alive():
                    thread.join(timeout=0.2)
        except KeyboardInterrupt:
            self.stopped.set()
            for thread in threads:
                thread.join(timeout=5)
            raise
        return 0 if self.failed == 0 else 1

    def summary(self) -> dict[str, Any]:
        elapsed = round(time.monotonic() - self.started, 3)
        attempted = len(self.episodes)
        succeeded = self.succeeded
        stages: dict[str, dict[str, int]] = {}
        for name in ("grasp", "place", "release", "stable"):
            reached = sum(
                1 for item in self.episodes if item.get("milestones", {}).get(name)
            )
            stages[name] = {"reached": reached, "total": attempted}
        failures: dict[str, dict[str, Any]] = {}
        for item in self.episodes:
            if item["succeeded"]:
                continue
            key = item.get("failure_stage") or "未分类"
            bucket = failures.setdefault(
                key, {"count": 0, "reasons": {}, "seeds": []}
            )
            bucket["count"] += 1
            reason = item.get("failure_reason") or "无原因"
            bucket["reasons"][reason] = bucket["reasons"].get(reason, 0) + 1
            bucket["seeds"].append(item["seed"])
        return {
            "schema_version": 1,
            "batch_id": self.args.batch_id,
            "requested": self.args.count,
            "attempted": attempted,
            "succeeded": succeeded,
            "failed": self.failed,
            "success_rate_percent": round(succeeded / attempted * 100.0, 2)
            if attempted
            else 0.0,
            "workers": len(self.workers),
            "seed_start": self.args.seed_start,
            "elapsed_s": elapsed,
            "stopped_early": self.stopped.is_set(),
            "stages": stages,
            "failure_groups": failures,
            "episodes": self.episodes,
        }


def _stage_label(name: str) -> str:
    return {
        "grasp": "抓取/交接",
        "place": "搬运/放置",
        "release": "释放/撤离",
        "stable": "稳定确认",
    }[name]


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--manifest", type=Path, required=True)
    parser.add_argument("--recordings", type=Path, required=True)
    parser.add_argument("--runs-root", type=Path, required=True)
    parser.add_argument("--console-root", type=Path, required=True)
    parser.add_argument("--report", type=Path, required=True)
    parser.add_argument("--batch-id", required=True)
    parser.add_argument("--count", type=int, required=True)
    parser.add_argument("--seed-start", type=int, required=True)
    parser.add_argument("--max-consecutive-failures", type=int, default=None)
    args = parser.parse_args(argv)
    document = json.loads(args.manifest.read_text(encoding="utf-8"))
    workers = document.get("workers")
    if not isinstance(workers, list) or not workers:
        print("inference: worker manifest has no workers", file=sys.stderr)
        return 2
    args.recordings.mkdir(parents=True, exist_ok=True)
    args.runs_root.mkdir(parents=True, exist_ok=True)
    args.console_root.mkdir(parents=True, exist_ok=True)
    pool = InferencePool(args, workers)
    print(
        f"并行推理开始：{len(workers)} 个 worker | 共 {args.count} 局 | "
        f"起始 seed {args.seed_start}",
        flush=True,
    )
    code = pool.run()
    summary = pool.summary()
    args.report.parent.mkdir(parents=True, exist_ok=True)
    args.report.write_text(
        json.dumps(summary, indent=2, ensure_ascii=False) + "\n", encoding="utf-8"
    )
    args.manifest.with_name("summary.json").write_text(
        json.dumps(summary, indent=2, ensure_ascii=False) + "\n", encoding="utf-8"
    )
    print("=== 并行推理结果 ===", flush=True)
    print(
        f"完成 {summary['attempted']}/{summary['requested']} 局 | "
        f"成功 {summary['succeeded']} | 失败 {summary['failed']} | "
        f"成功率 {summary['success_rate_percent']:.2f}% | "
        f"用时 {_format_duration(summary['elapsed_s'])}",
        flush=True,
    )
    for name in ("grasp", "place", "release", "stable"):
        stage = summary["stages"][name]
        rate = (
            stage["reached"] / stage["total"] * 100.0 if stage["total"] else 0.0
        )
        print(
            f"  {_stage_label(name)} 达成 {stage['reached']}/{stage['total']} "
            f"({rate:.0f}%)",
            flush=True,
        )
    if summary["failure_groups"]:
        print("失败分布:", flush=True)
        for stage, group in summary["failure_groups"].items():
            reasons = "; ".join(
                f"{reason} ×{count}" for reason, count in group["reasons"].items()
            )
            print(f"  {stage}: {group['count']} 次（{reasons}）", flush=True)
    print(f"报告: {args.report}", flush=True)
    return code


if __name__ == "__main__":
    raise SystemExit(main())
