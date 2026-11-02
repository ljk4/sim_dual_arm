# Native bounded-follower execution

DSH-031 的公共记录、`SubmitFollower` 服务和可选后端入口见
[native-follower-protocol.md](native-follower-protocol.md)。

DSH-030 connects the accepted bounded-target follower math (DSH-029) to the
same native physical loop that executes strict chunks.  It does not add a second
controller, a second physics loop, stacking, trial collection or training.

## Concrete stream types

`components/backend_sim/src/robot_backend_sim/chunk_types.py` defines the
explicit union

```python
type PreparedStreamChunk = PreparedChunk | PreparedFollowerPlan
```

`chunk_kind` decides `CHUNK` / `FOLLOWER` with `isinstance` against those two
real classes.  The follower does **not** inherit from `PreparedChunk` and a
duck-typed object is never accepted by the production controller/motion path.
`StreamSchedule` keeps a small legacy probe for the old attribute-compatible
unit fixtures; new production values still have to be one of the two concrete
types.

`PreparedFollowerPlan` adds one derived read-only property, `settle_at_end`,
which returns the real `is_final` flag.  No `final_velocities` alias is added;
follower terminal data continues to use `raw_terminal_velocities`,
`restricted_terminal_velocities` and `actual_terminal_*`.

## Commands and fingerprints

`PreparedControlCommand.kind` now returns `MOTION`, `CHUNK` or `FOLLOWER`.
The follower packet uses a separate payload and may keep `request=None`; it does
not fabricate a strict `AcceptedReference`.  The existing `NativeRequest`
validation for MOTION/CHUNK packets is unchanged.

The follower fingerprint covers:

* `FOLLOWER_ALGORITHM_VERSION`;
* the complete anchor, including its position/velocity/acceleration reference;
* the complete source window and the selected relative targets;
* every exported cubic coefficient and segment duration;
* joints, `max_accelerations` and the configured deviation limits;
* raw and restricted terminal velocities;
* actual terminal position/velocity/acceleration, deviation evidence and flags;
* requested window, real Ruckig duration, extension and optional grid.

MOTION and CHUNK payload construction is untouched.  The fixed fixture hashes
asserted by `tests/unit/test_follower_control_state.py` are byte-identical to the
pre-DSH-030 implementation.

## Stream scheduling and hand-off

The first follower chunk goes through `NativeController.submit`; every successor
goes through `queue_chunk`.  Both calls stay on the controller's owning thread
and reuse `NativeMotion`, `StreamSchedule`, the lease book and command/chunk
history.  A stream has one active chunk and at most one pending successor.

Strict and follower chunks cannot be mixed in one stream.  The controller
rejects a mixed successor with `STREAM_MODE_MISMATCH`; a new stream after reset
may choose either mode.  Mode mixing is an invalid package, so it is rejected
without replacing the currently valid active chunk.

A successor anchor is taken from the active plan's actual endpoint sample
(position and velocity), never from the raw target and never from a measured
state.  Every original source target (60 x 16 in the physical test) and the
observation time are retained in the `ActionWindow`.  Non-final plans end
exactly at the fixed hand-off; they do not settle between chunks.

## Physical duration and final settling

`prepare_follower_plan(..., duration_grid_ns=...)` defaults to `None`, preserving
the accepted continuous DSH-029 behavior.  Physical callers pass
`2_000_000` ns (the scene step).  In that case:

* `InputParameter.duration_discretization` is `Discrete`;
* `Ruckig(count, delta_time=grid_ns / 1e9)` calculates at the original
  `minimum_duration`;
* a non-final fixed hand-off must lie exactly on the grid;
* the exported plan duration must be an exact multiple of the grid;
* the 3 s shared horizon and configured terminal-tail bound still apply.

The Ruckig duration metadata is read from the calculated trajectory; it is never
hand-edited.  A final plan reaches the raw last selected target at zero terminal
velocity and then the existing physical settling/contact logic waits for real
measurement stability.  The follower final CONTACT path keeps the raw closing
target in the completion bounds; completion is decided by measured finger
contact/force/opening, not by position arrival.

## Numeric hard-boundary normalization

The shared normalization lives in
`robot_contracts.follower_math.normalize_follower_positions`.  It is applied to
follower reference positions immediately before `PandaIO.apply_targets`:

* values already inside `[minimum, maximum]` are unchanged;
* a value outside a hard boundary by at most
  `LIMIT_ATOL + LIMIT_RTOL * |bound|` is moved exactly to that boundary;
* a larger excursion raises `FOLLOWER_BOUNDARY_MISMATCH` and is not silently
  clipped.

For each application the motion snapshot retains the untouched coefficient
sample, the actual applied reference and the per-axis correction
(`applied = sampled - correction`).  The same function is used by tests to
rebuild the applied reference from the saved coefficients.  Only positions are
normalized because only positions are written by `PandaIO.apply_targets`;
velocities and accelerations keep their original values and the shared
continuous-limit checks.  The strict `PreparedChunk` path is not normalized and
`PandaIO`'s hard limit checks are not relaxed.

## Failure behavior

The new mode is subject to the same lease/context/profile checks, `accept_before`,
observation age, `minLead`, C1, hard limits, history bounds and stop deadlines.
Late prediction, underrun, tracking timeout and lease expiry enter the existing
braking state machine (`reference_kind="BRAKE"`) and finish with an explicit
reason.  A modified retry with the same command/chunk identity but a different
fingerprint is rejected as a conflict; an identical-fingerprint retry is
idempotent.

## Verification

* `tests/unit/test_follower_control_state.py` covers the concrete kind union,
  fixed legacy fingerprints, complete follower fingerprint coverage, grid
  validation and positive/negative/out-of-tolerance normalization.
* `tests/integration/test_follower_native.py` drives the real MuJoCo scene with
  eleven continuous follower chunks, checks active/pending/retired/terminal
  status, independently rebuilds every coefficient-sampled reference from saved
  plans, verifies the final 2 ms-grid extension and measured stability, and
  exercises strict/follower comparison, underrun, late prediction, mode mixing,
  retries, context/reset, lease expiry, raw/deviation rejection, numeric
  boundary I/O and follower CONTACT.

ROS is not connected in this worktree.  The ROS-side tests and the Follower
service that populates packets are left to the supervisor / next item.  No
stacking, trial collection, training, scene change or hard-limit relaxation is
included.
