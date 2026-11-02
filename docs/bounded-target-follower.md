# Bounded target follower: pure math and comparison

Status: math/config/DTO layer only.  This change does **not** modify the ROS IDL,
controller, worker, runtime or training code, and it does not claim that a
physical or dual-arm task has been executed.

## Scope

The accepted strict path (`ActionWindow` -> `prepare_chunk` -> `PreparedChunk`)
still fits every selected original prediction exactly.  It remains unchanged.
This document describes a separate bounded-follower experiment:

* the original ACT prediction (the complete `ActionWindow.source_targets`) is
  retained without changing any position;
* the actual executed reference is a separate explicit cubic
  `ReferencePlan`;
* raw selected targets, desired piecewise-linear targets, actual reference,
  terminal velocities and continuous target deviations are all kept;
* no measured state is used to re-base a reference;
* no object ground truth is used.

The implementation lives in:

| File | Responsibility |
| --- | --- |
| `contracts/python/src/robot_contracts/follower.py` | Standard-library-only `PreparedFollowerPlan`, limit flags and exact target-deviation math. |
| `components/backend_sim/src/robot_backend_sim/ruckig_export.py` | Public Ruckig phase arrays -> explicit integer-nanosecond cubic segments. |
| `components/backend_sim/src/robot_backend_sim/follower_plan.py` | One Ruckig call, terminal-speed restriction, truncation/extension and validation. |
| `components/backend_sim/src/robot_backend_sim/follower_config.py` | Strict immutable follower configuration. |
| `configs/simulation/panda_follower.yaml` | Initial engineering parameters. |

`PreparedFollowerPlan` is intentionally **not** a subclass or alias of
`PreparedChunk`.  The shared DTO never imports Ruckig, NumPy, SciPy or ROS.

## Inputs and output

Inputs reuse the accepted contracts:

* `ActionWindow`, including all 60 source predictions at 20 Hz for the initial
  engineering case;
* `ChunkAnchor`, whose reference position/velocity is the fixed hand-off state;
* `ReferencePlan`, `CubicSegment`, `JointSpec` and `PandaControlConfig`;
* `PandaFollowerConfig` for `target_count`, per-axis deviation budgets and the
  terminal-tail limit.

`PreparedFollowerPlan` stores:

* the original `anchor` and `window`;
* the explicit coefficient `plan`;
* `raw_terminal_velocities` (backward difference of the last two selected raw
  predictions);
* `restricted_terminal_velocities` (what Ruckig was asked to target);
* `actual_terminal_positions`, `actual_terminal_velocities` and
  `actual_terminal_accelerations` (the real exported curve endpoint);
* `max_target_deviations` and `max_target_deviation_limits`;
* `FollowerLimitFlags` with per-axis `velocity_limited`,
  `acceleration_limited` and `boundary_preserved`;
* `requested_window_ns`, `ruckig_duration_ns`, `extension_ns`, and the real
  `plan.duration_ns`.

`to_document()` emits a readable JSON-serialisable summary with the complete
source targets, desired targets, coefficient segments, extrema and flags.  It is
a comparison artifact, not a claim of physical success.

## Planning method

1. Validate the full source window, anchor, joint order and configured limits.
2. Select `target_count` predictions after the hand-off through `ActionWindow`;
   initial parameters are five points / 0.25 s.
3. Take the last selected raw position as Ruckig's target position.  The raw
   position is never modified.
4. Compute raw terminal velocity from the selected last two points.
5. Restrict only the terminal speed when necessary.  The single generic rule
   lives in `robot_contracts.follower.restrict_terminal_velocities` and is used
   by both the backend adapter and the immutable DTO.  For displacement
   direction `s`, distance `d`, initial signed speed `u0`, acceleration limit
   `a`, joint minimum `min` and maximum `max`:

   ```text
   approach_cap = sqrt(max(0, u0^2 + 2*a*d))   if u0 >= 0
   approach_cap = sqrt(max(0, 2*a*d - u0^2))   otherwise
   signed = min(max(s*raw, 0), approach_cap, max_velocity)

   if signed > 0:
       signed = min(signed, sqrt(2*a*(max - goal)))
   elif signed < 0:
       signed = max(signed, -sqrt(2*a*(goal - min)))
   ```

   The second pair is applied to every `JointSpec`, not just the gripper.  It
   ensures the terminal speed at the goal can still brake to zero without
   crossing the true joint position hard boundary.  A terminal speed opposing
   the total displacement is reduced to zero.  The original raw value and the
   limited value are both stored.
6. Call Ruckig Community 0.15.3 exactly once with second-order limits
   (`max_jerk = inf`, no waypoints, no Pro/cloud API) and
   `minimum_duration = window.end_time_ns / 1e9`.
7. Export the public phase arrays to explicit coefficients.
8. Non-final: truncate exactly at the fixed hand-off
   (`window.end_time_ns`).  The actual endpoint may not have reached the raw
   target; its exported position/velocity is what a successor must inherit.
9. Final: target terminal velocity is zero.  If Ruckig needs longer than the
   requested window it extends the reference until the original last target is
   reached.  The total duration must not exceed the shared 3 s horizon or the
   configured terminal tail.  `extension_ns` records the extension.
10. Validate continuous position/velocity/acceleration limits, C1 seams, the
    actual target-deviation budget and endpoint braking feasibility.  The
    immutable shared DTO independently derives `raw_terminal_velocities` from
    the window's last two targets, checks `restricted_terminal_velocities`
    against the same generic rule, and analytically checks the actual endpoint
    stop distance `v^2/(2a)` against every `JointSpec` bound.  Failure is an
    explicit `ContractError`; constraints are never silently relaxed and a
    non-final truncation is never reported as arrival.

The exact reasons include `INVALID_FOLLOWER_REQUEST`,
`FOLLOWER_RUCKIG_REJECTED`, `FOLLOWER_INFEASIBLE`,
`FOLLOWER_HORIZON_EXCEEDED`, `FOLLOWER_BRAKING_INFEASIBLE` and
`FOLLOWER_DEVIATION_EXCEEDED`.

## Coefficient export

Ruckig exposes one start value per phase plus an extra final value for
position/velocity/acceleration.  For every positive-duration phase:

```text
c0 = p
c1 = v
c2 = a / 2
c3 = j / 6
```

Phase boundaries from all axes are merged.  Durations are rounded to integer
nanoseconds; the merged coefficients use seconds relative to each rounded
boundary.  At a seam the next segment starts from the previous exported
position and velocity, so the stored curve is exactly C1; acceleration may step.
The measured position/velocity error against `trajectory.at_time` away from
seams is at the few-`1e-10` level for the initial tests.

Handled structures:

* zero-duration phases are skipped;
* axes with different phase boundaries are merged;
* stationary axes use a constant hold phase;
* direction reversal and deceleration/braking phases are ordinary phases with
  the signs supplied by Ruckig;
* a nonzero public `profile.brake` is an unsupported structure and is rejected
  explicitly;
* multiple profile sections (which would imply intermediate waypoints) are
  rejected explicitly.

Truncation is a coefficient export at the fixed integer-nanosecond boundary; it
is not a 20 Hz resampling.

## Desired target and exact deviation

The desired target starts at the anchor position at `t = 0`, is piecewise linear
through the selected raw predictions at their hand-off-relative times, and for a
final plan holds the last raw value on the extension.

For each intersection of a reference segment and a desired segment, the
difference is a cubic in local seconds.  Its derivative is a quadratic; real
roots plus the interval endpoints give the exact maximum absolute deviation.
The implementation never substitutes control-tick sampling for this proof.  A
deviation above the configured per-axis array raises
`FOLLOWER_DEVIATION_EXCEEDED`.

## Configuration

`configs/simulation/panda_follower.yaml` contains:

* the fixed 16-joint order;
* `target_count: 5`;
* `max_target_deviations`: `0.15` rad for each arm joint and `0.02` m for each
  total gripper opening;
* `max_terminal_tail_duration_s: 3.0`.

Parsing rejects unknown keys, duplicate keys, missing keys, booleans, NaN,
infinity, wrong array lengths and non-positive values.  These deviation budgets
are following budgets and do not widen the robot profile's joint position or
velocity hard limits.

## Verification performed

The committed unit tests cover:

* standard-library DTO immutability and serialisation;
* strict configuration parsing;
* static, single-axis, 16-axis, reverse, tiny-time and gripper-boundary export;
* comparison of the exported curve with the public Ruckig trajectory at dense
  deterministic times and on both sides of every merged seam;
* 100 deterministic random 16-axis feasible cases with independent continuous
  limit and C1 checks;
* the strict first-window counterexample `.04 rad / 3 s` smoothstep with a
  250 ms lead, where `prepare_chunk` rejects `maxa = 1.8284756 > 1`, while the
  bounded follower produces a legal reference within budget;
* `2 rad / 7 s` fragments with 50, 100 and 250 ms leads;
* exact cubic deviation, exceeded-budget rejection and inheritance of the actual
  non-final endpoint by the successor;
* both gripper hard-boundary directions at 0.002 m and 0.078 m, including the
  supervisor's `.0016/.0012/.0008/.0004/0` and reverse examples;
* successive windows converging on a joint minimum or maximum with inherited
  actual p/v, and a 16-axis pure rule test;
* forged `raw_terminal_velocities`/`restricted_terminal_velocities` metadata
  rejection and analytic endpoint-braking rejection.

The repository tests run with the task-provided project interpreter and the
Ruckig site-packages path, for example:

```bash
PYTHONPATH=/home/ljk/work/sim_dual_arm/.build/follower-probe-venv/lib/python3.12/site-packages \
/home/ljk/work/sim_dual_arm/.build/native-dev-venv/bin/python -m pytest \
  tests/unit/test_follower_contracts.py tests/unit/test_follower_plan.py
```

Ruff uses `/home/ljk/work/sim_dual_arm/.build/dev-venv/bin/ruff`.

纯标准库层按职责分为：`robot_contracts/follower.py`保存不可变记录及其校验，`follower_math.py`保存端速约束与连续偏差计算。原有公共导入路径保留。
