"""Pure-NumPy servo dynamics identification (delay, step clamp, gripper deadband)."""
from __future__ import annotations

import argparse
import json
import time
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any, Sequence

import numpy as np
import yaml

from robot_backend_hardware.common.gripper import GripperCommand
from robot_backend_hardware.tianji.assembly import build_devices
from robot_backend_hardware.tianji.config import load_config
from robot_backend_hardware.tianji.types import CmdKey, CommandSpace, MotionMode, StateKey


DEFAULT_OUTPUT_PATH = Path("/data/_system/debug/tianji_calibration/sysid_result.json")
MAX_EXCITATION_AMPLITUDE_DEG = 4.0
MAX_EXCITATION_FREQ_HZ = 0.8


@dataclass(frozen=True)
class SysIdResult:
    """Identified servo dynamics matching simulation/mujoco ServoDynamicsConfig."""

    control_dt_s: float
    arm_delay_steps: int
    gripper_delay_steps: int
    max_joint_step_rad: float
    gripper_deadband_m: float
    arm_tracking_rmse_rad: float
    per_joint_delay_steps: list[int]
    servo_dynamics_yaml: str


def _estimate_lag_steps(cmd: np.ndarray, fdb: np.ndarray, max_lag: int) -> int:
    """Estimate causal integer lag d >= 0 aligning cmd(t) with fdb(t + d * dt)."""
    d_cmd = np.diff(cmd)
    d_fdb = np.diff(fdb)
    max_d = min(max_lag, max(0, len(d_cmd) - 3))
    best_lag = 0
    best_score = -float("inf")
    for d in range(max_d + 1):
        a = d_cmd[: len(d_cmd) - d] if d > 0 else d_cmd
        b = d_fdb[d:]
        denom = float(np.linalg.norm(a) * np.linalg.norm(b))
        corr = float(np.dot(a, b) / denom) if denom > 1e-12 else -1.0
        # Break near-ties using negative L2 velocity residual.
        err = float(np.mean((a - b) ** 2))
        score = corr - 0.01 * err
        if score > best_score:
            best_score = score
            best_lag = d
    return best_lag


def _identify_gripper_params(
    gripper_timestamps_s: Sequence[float] | np.ndarray,
    g_cmd_m: Sequence[float] | np.ndarray,
    g_fdb_m: Sequence[float] | np.ndarray,
    *,
    control_dt_s: float,
) -> tuple[int, float]:
    t = np.asarray(gripper_timestamps_s, dtype=np.float64)
    cmd = np.asarray(g_cmd_m, dtype=np.float64)
    fdb = np.asarray(g_fdb_m, dtype=np.float64)
    if t.size < 4 or cmd.size != t.size or fdb.size != t.size:
        raise ValueError("Gripper sys-id arrays must have equal length >= 4")

    t0, t1 = float(t[0]), float(t[-1])
    grid = np.arange(t0, t1 + 1e-9, control_dt_s, dtype=np.float64)
    cmd_g = np.interp(grid, t, cmd)
    fdb_g = np.interp(grid, t, fdb)

    deltas = np.abs(np.diff(cmd_g))
    step_indices = np.flatnonzero(deltas > 1e-6)
    suppressed: list[float] = []
    moving_deltas: list[float] = []
    lags: list[int] = []

    for idx in step_indices:
        mag = float(deltas[idx])
        step_idx = int(idx) + 1
        before = float(fdb_g[idx])
        window_end = min(len(fdb_g), step_idx + max(6, int(round(0.4 / control_dt_s))))
        after_slice = fdb_g[step_idx:window_end]
        moved = float(np.max(np.abs(after_slice - before)))
        if moved < 0.25 * mag:
            suppressed.append(mag)
        else:
            moving_deltas.append(mag)
            target_jump = abs(float(cmd_g[step_idx]) - before)
            thresh = max(1e-4, 0.15 * target_jump)
            hit = np.flatnonzero(np.abs(after_slice - before) >= thresh)
            if hit.size > 0:
                lags.append(int(hit[0]))

    delay_steps = int(round(float(np.median(lags)))) if lags else 2
    if suppressed and moving_deltas:
        max_suppressed = max(suppressed)
        min_moving = min(moving_deltas)
        if min_moving <= 4.0 * max_suppressed:
            deadband_m = 0.5 * (max_suppressed + min_moving)
        else:
            deadband_m = 1.25 * max_suppressed
    elif suppressed:
        deadband_m = 1.25 * max(suppressed)
    else:
        deadband_m = 0.0003
    return delay_steps, float(round(deadband_m, 6))


def fit_servo_dynamics(
    timestamps_s: Sequence[float] | np.ndarray,
    q_cmd_rad: Sequence[Sequence[float]] | np.ndarray,
    q_fdb_rad: Sequence[Sequence[float]] | np.ndarray,
    *,
    control_dt_s: float = 0.02,
    gripper_timestamps_s: Sequence[float] | np.ndarray | None = None,
    g_cmd_m: Sequence[float] | np.ndarray | None = None,
    g_fdb_m: Sequence[float] | np.ndarray | None = None,
) -> SysIdResult:
    """Fit discrete control delay, per-step joint bound, and gripper deadband from logs."""
    t = np.asarray(timestamps_s, dtype=np.float64)
    cmd = np.asarray(q_cmd_rad, dtype=np.float64)
    fdb = np.asarray(q_fdb_rad, dtype=np.float64)
    if t.ndim != 1 or t.size < 6 or cmd.shape != fdb.shape or cmd.shape[0] != t.size:
        raise ValueError("Invalid arm sys-id trajectory shapes")
    if control_dt_s <= 0.0:
        raise ValueError("control_dt_s must be positive")

    grid = np.arange(float(t[0]), float(t[-1]) + 1e-9, control_dt_s, dtype=np.float64)
    n_joints = cmd.shape[1]
    cmd_g = np.column_stack([np.interp(grid, t, cmd[:, j]) for j in range(n_joints)])
    fdb_g = np.column_stack([np.interp(grid, t, fdb[:, j]) for j in range(n_joints)])

    per_joint_delays: list[int] = []
    active_delays: list[int] = []
    for j in range(n_joints):
        d = _estimate_lag_steps(cmd_g[:, j], fdb_g[:, j], max_lag=12)
        per_joint_delays.append(d)
        if float(np.ptp(cmd_g[:, j])) > 1e-4:
            active_delays.append(d)

    arm_delay = int(round(float(np.median(active_delays)))) if active_delays else 0
    if arm_delay > 0 and len(cmd_g) > arm_delay:
        rmse = float(np.sqrt(np.mean((cmd_g[:-arm_delay] - fdb_g[arm_delay:]) ** 2)))
    else:
        rmse = float(np.sqrt(np.mean((cmd_g - fdb_g) ** 2)))

    cmd_aligned = cmd_g[:-arm_delay] if arm_delay > 0 and len(cmd_g) > arm_delay else cmd_g
    fdb_aligned = fdb_g[arm_delay:] if arm_delay > 0 and len(cmd_g) > arm_delay else fdb_g
    req_delta = np.abs(cmd_aligned[1:] - fdb_aligned[:-1])
    act_step = np.abs(fdb_aligned[1:] - fdb_aligned[:-1])
    clamped = (req_delta > 1.25 * act_step) & (act_step > 1e-4)
    if np.any(clamped):
        max_step = float(np.median(act_step[clamped]))
    else:
        active_cols = [j for j in range(n_joints) if float(np.ptp(cmd_g[:, j])) > 1e-4]
        active_steps = act_step[:, active_cols] if active_cols else act_step
        max_step = float(np.percentile(active_steps, 99.0)) if active_steps.size else 0.05

    if gripper_timestamps_s is not None and g_cmd_m is not None and g_fdb_m is not None:
        g_delay, g_deadband = _identify_gripper_params(
            gripper_timestamps_s, g_cmd_m, g_fdb_m, control_dt_s=control_dt_s
        )
    else:
        g_delay, g_deadband = 2, 0.0003

    yaml_snippet = yaml.safe_dump(
        {
            "servo_dynamics": {
                "enabled": True,
                "arm_delay_steps": arm_delay,
                "gripper_delay_steps": g_delay,
                "max_joint_step_rad": round(max_step, 4),
                "gripper_deadband_m": round(g_deadband, 6),
            }
        },
        sort_keys=False,
    )
    return SysIdResult(
        control_dt_s=float(control_dt_s),
        arm_delay_steps=arm_delay,
        gripper_delay_steps=g_delay,
        max_joint_step_rad=float(round(max_step, 5)),
        gripper_deadband_m=float(round(g_deadband, 6)),
        arm_tracking_rmse_rad=rmse,
        per_joint_delay_steps=per_joint_delays,
        servo_dynamics_yaml=yaml_snippet,
    )


def _run_live_airborne_excitation(
    config_path: Path,
    side: str,
    *,
    amplitude_deg: float,
    freq_hz: float,
    duration_s: float,
    control_dt_s: float,
) -> dict[str, Any]:
    if not (0.0 < amplitude_deg <= MAX_EXCITATION_AMPLITUDE_DEG):
        raise ValueError(f"amplitude_deg={amplitude_deg} exceeds {MAX_EXCITATION_AMPLITUDE_DEG}")
    if not (0.0 < freq_hz <= MAX_EXCITATION_FREQ_HZ):
        raise ValueError(f"freq_hz={freq_hz} exceeds {MAX_EXCITATION_FREQ_HZ}")

    cfg = load_config(config_path)
    devices = build_devices(cfg)
    devices.start_all()
    try:
        arms = devices.arms
        gripper = devices.grippers[side]
        cmd_key = CmdKey.LEFTARM_CMD_JOINT_POS if side == "left" else CmdKey.RIGHTARM_CMD_JOINT_POS
        state_key = StateKey.LEFTARM_STATE_JOINT_POS if side == "left" else StateKey.RIGHTARM_STATE_JOINT_POS

        q0_deg: np.ndarray | None = None
        for _ in range(50):
            st = arms.poll_state()
            if state_key in st:
                q0_deg = st[state_key].copy()
                break
            time.sleep(0.02)
        if q0_deg is None:
            raise RuntimeError(f"No live UDP state for {side} arm")

        arms.set_motion_mode(MotionMode.ONLINE_TRACKING, CommandSpace.JOINT)
        ts_list: list[float] = []
        q_cmd_list: list[list[float]] = []
        q_fdb_list: list[list[float]] = []

        t0 = time.monotonic()
        step = 0
        while True:
            now = time.monotonic() - t0
            if now >= duration_s:
                break
            offset = amplitude_deg * np.sin(2.0 * np.pi * freq_hz * now)
            target_deg = q0_deg.copy()
            target_deg[5] += offset
            target_deg[6] += offset
            arms.send_cmd({cmd_key: target_deg})
            st = arms.poll_state()
            if state_key in st:
                ts_list.append(now)
                q_cmd_list.append(np.radians(target_deg).tolist())
                q_fdb_list.append(np.radians(st[state_key]).tolist())
            step += 1
            sleep_for = t0 + step * control_dt_s - time.monotonic()
            if sleep_for > 0.0:
                time.sleep(sleep_for)

        arms.set_motion_mode(MotionMode.TRAJECTORY, CommandSpace.JOINT)

        # Safe gripper micro-step sweep
        g_ts: list[float] = []
        g_cmd: list[float] = []
        g_fdb: list[float] = []
        gt0 = time.monotonic()
        for target_mm in (60.0, 60.2, 60.6, 55.0, 75.0, 60.0):
            gripper.set(GripperCommand(position_mm=target_mm, speed_pct=20.0, force_pct=25.0))
            sub_end = time.monotonic() + 0.5
            while time.monotonic() < sub_end:
                gst = gripper.get()
                if gst.valid:
                    g_ts.append(time.monotonic() - gt0)
                    g_cmd.append(target_mm / 1000.0)
                    g_fdb.append(gst.position_mm / 1000.0)
                time.sleep(control_dt_s)
    finally:
        devices.stop_all()

    return {
        "timestamps_s": ts_list,
        "q_cmd_rad": q_cmd_list,
        "q_fdb_rad": q_fdb_list,
        "gripper_timestamps_s": g_ts,
        "g_cmd_m": g_cmd,
        "g_fdb_m": g_fdb,
    }


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=Path("/app/configs/devices.yaml"))
    parser.add_argument("--arm", choices=("left", "right"), default="right")
    parser.add_argument("--control-dt-s", type=float, default=0.02)
    parser.add_argument("--amplitude-deg", type=float, default=3.0)
    parser.add_argument("--freq-hz", type=float, default=0.5)
    parser.add_argument("--duration-s", type=float, default=6.0)
    parser.add_argument("--from-log", type=Path, default=None, help="Offline Sys-ID JSON trajectory")
    parser.add_argument("--live", action="store_true", help="Execute bounded airborne excitation on live hardware")
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT_PATH)
    args = parser.parse_args(argv)

    print(
        f"[run_sysid] config={args.config} arm={args.arm} dt={args.control_dt_s}s "
        f"live={args.live} output={args.output}"
    )

    if args.from_log is not None:
        raw = json.loads(args.from_log.read_text(encoding="utf-8"))
    elif args.live:
        raw = _run_live_airborne_excitation(
            args.config,
            args.arm,
            amplitude_deg=args.amplitude_deg,
            freq_hz=args.freq_hz,
            duration_s=args.duration_s,
            control_dt_s=args.control_dt_s,
        )
    else:
        raise SystemExit("Pass either --from-log <path> (offline) or --live (hardware)")

    res = fit_servo_dynamics(
        raw["timestamps_s"],
        raw["q_cmd_rad"],
        raw["q_fdb_rad"],
        control_dt_s=args.control_dt_s,
        gripper_timestamps_s=raw.get("gripper_timestamps_s"),
        g_cmd_m=raw.get("g_cmd_m"),
        g_fdb_m=raw.get("g_fdb_m"),
    )
    payload = asdict(res)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(payload, indent=2, sort_keys=True), encoding="utf-8")
    print(
        f"[run_sysid] arm_delay={res.arm_delay_steps}steps gripper_delay={res.gripper_delay_steps}steps "
        f"max_joint_step={res.max_joint_step_rad:.4f}rad deadband={res.gripper_deadband_m:.6f}m"
    )
    print(res.servo_dynamics_yaml, end="")
    print(f"[run_sysid] saved {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
