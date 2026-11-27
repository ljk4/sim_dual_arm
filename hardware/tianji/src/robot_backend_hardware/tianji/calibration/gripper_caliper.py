"""Interactive gripper caliper probe for CTAG2F90C + tactile silicone pads."""
from __future__ import annotations

import argparse
import json
import time
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Mapping, Sequence

from robot_backend_hardware.common.gripper import GripperCommand, TwoFingerGripper
from robot_backend_hardware.tianji.assembly import build_devices
from robot_backend_hardware.tianji.config import load_config


DEFAULT_OUTPUT_PATH = Path("/data/_system/debug/tianji_calibration/gripper_caliper.json")
MAX_SAFE_SPEED_PCT = 20.0
MAX_SAFE_FORCE_PCT = 25.0


@dataclass(frozen=True)
class CaliperResult:
    """Calibrated tactile-pad closure zero-offset and measured object widths."""

    side: str
    empty_touch_position_mm: float
    pad_inward_offset_m: float
    max_stroke_mm: float
    measured_widths_m: dict[str, float]


def compute_caliper_calibration(
    side: str,
    empty_touch_position_mm: float,
    object_touch_positions_mm: Mapping[str, float] | None = None,
    *,
    max_stroke_mm: float = 90.0,
) -> CaliperResult:
    """Compute pad_inward_offset_m and object widths from CTAG2F90C touch positions.

    With invert_stroke=True, position_mm reports jaw-holder opening in [0, 90] mm.
    When opposing soft pads meet with nothing between them at position_mm > 0, each
    pad protrudes inward by half of that remaining jaw stroke:
      pad_inward_offset_m = (empty_touch_position_mm / 2.0) * 1e-3
    Any grasped object's compressed thickness equals (object_pos - empty_pos) * 1e-3.
    """
    if max_stroke_mm <= 0.0:
        raise ValueError(f"max_stroke_mm must be positive, got {max_stroke_mm}")
    if not (0.0 <= empty_touch_position_mm < max_stroke_mm):
        raise ValueError(
            f"empty_touch_position_mm={empty_touch_position_mm} outside [0, {max_stroke_mm})"
        )

    widths_m: dict[str, float] = {}
    for obj_name, pos_mm in (object_touch_positions_mm or {}).items():
        if not (-0.5 <= pos_mm <= max_stroke_mm):
            raise ValueError(f"Object {obj_name!r} touch position {pos_mm} out of range")
        width_mm = max(0.0, float(pos_mm) - float(empty_touch_position_mm))
        widths_m[str(obj_name)] = width_mm / 1000.0

    return CaliperResult(
        side=side,
        empty_touch_position_mm=float(empty_touch_position_mm),
        pad_inward_offset_m=float(empty_touch_position_mm) / 2000.0,
        max_stroke_mm=float(max_stroke_mm),
        measured_widths_m=widths_m,
    )


def probe_gripper_touch(
    gripper: TwoFingerGripper,
    *,
    target_mm: float = 0.0,
    speed_pct: float = 10.0,
    force_pct: float = 15.0,
    poll_hz: float = 25.0,
    timeout_s: float = 8.0,
    reopen_mm: float = 85.0,
) -> float:
    """Safely close a gripper at low speed/torque until pad contact, record reading, and reopen."""
    if not (0.0 < speed_pct <= MAX_SAFE_SPEED_PCT):
        raise ValueError(f"speed_pct={speed_pct} exceeds safe limit (0, {MAX_SAFE_SPEED_PCT}]")
    if not (0.0 < force_pct <= MAX_SAFE_FORCE_PCT):
        raise ValueError(f"force_pct={force_pct} exceeds safe limit (0, {MAX_SAFE_FORCE_PCT}]")

    period_s = 1.0 / max(poll_hz, 1.0)
    last_pos_mm: float | None = None
    deadline = time.monotonic() + timeout_s
    started = time.monotonic()
    try:
        gripper.set(
            GripperCommand(
                position_mm=float(target_mm),
                speed_pct=float(speed_pct),
                force_pct=float(force_pct),
            )
        )
        while time.monotonic() < deadline:
            state = gripper.get()
            if state.valid:
                last_pos_mm = float(state.position_mm)
                elapsed = time.monotonic() - started
                if state.is_holding or (not state.is_moving and elapsed >= 0.4):
                    return last_pos_mm
            time.sleep(period_s)
        raise TimeoutError(f"Gripper {gripper.name!r} touch probe timed out after {timeout_s:.1f}s")
    finally:
        try:
            gripper.set(
                GripperCommand(
                    position_mm=float(reopen_mm),
                    speed_pct=float(speed_pct),
                    force_pct=float(force_pct),
                )
            )
        except Exception:
            pass


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=Path("/app/configs/devices.yaml"))
    parser.add_argument("--side", choices=("left", "right", "both"), default="both")
    parser.add_argument("--mode", choices=("empty", "object"), default="empty")
    parser.add_argument("--object-name", type=str, default="object")
    parser.add_argument("--speed-pct", type=float, default=10.0)
    parser.add_argument("--force-pct", type=float, default=15.0)
    parser.add_argument("--reopen-mm", type=float, default=85.0)
    parser.add_argument("--offline-empty-mm", type=float, default=None)
    parser.add_argument("--offline-object-mm", type=float, default=None)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT_PATH)
    args = parser.parse_args(argv)

    cfg = load_config(args.config)
    sides = ("left", "right") if args.side == "both" else (args.side,)
    print(
        f"[gripper_caliper] config={args.config.resolve()} sides={sides} mode={args.mode} "
        f"speed={args.speed_pct}% force={args.force_pct}% output={args.output}"
    )

    existing: dict = {}
    if args.output.exists():
        try:
            existing = json.loads(args.output.read_text(encoding="utf-8"))
        except Exception:
            existing = {}

    side_data = dict(existing.get("sides", {}))

    if args.offline_empty_mm is not None:
        for side in sides:
            objs = {args.object_name: args.offline_object_mm} if args.offline_object_mm is not None else {}
            res = compute_caliper_calibration(
                side,
                args.offline_empty_mm,
                objs,
                max_stroke_mm=getattr(cfg.grippers, side).max_stroke_mm,
            )
            side_data[side] = asdict(res)
    else:
        devices = build_devices(cfg)
        for side in sides:
            gripper = devices.grippers[side]
            gripper.start()
            try:
                pos_mm = probe_gripper_touch(
                    gripper,
                    speed_pct=args.speed_pct,
                    force_pct=args.force_pct,
                    reopen_mm=args.reopen_mm,
                )
            finally:
                gripper.stop()

            prev_side = side_data.get(side, {})
            if args.mode == "empty":
                empty_mm = pos_mm
                objs = {
                    name: round(empty_mm + width_m * 1000.0, 3)
                    for name, width_m in prev_side.get("measured_widths_m", {}).items()
                }
            else:
                empty_mm = float(prev_side.get("empty_touch_position_mm", 0.0))
                objs = {
                    name: round(empty_mm + width_m * 1000.0, 3)
                    for name, width_m in prev_side.get("measured_widths_m", {}).items()
                }
                objs[args.object_name] = pos_mm

            res = compute_caliper_calibration(
                side,
                empty_mm,
                objs,
                max_stroke_mm=gripper.max_stroke_mm,
            )
            side_data[side] = asdict(res)

    payload = {"sides": side_data}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(payload, indent=2, sort_keys=True), encoding="utf-8")
    for side, item in side_data.items():
        print(
            f"[gripper_caliper] {side}: empty_touch={item['empty_touch_position_mm']:.2f}mm "
            f"pad_inward_offset_m={item['pad_inward_offset_m']:.5f} "
            f"widths_m={item['measured_widths_m']}"
        )
    print(f"[gripper_caliper] saved {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
