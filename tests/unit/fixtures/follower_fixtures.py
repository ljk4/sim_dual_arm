"""Generate a test trajectory through the production command path."""

from robot_backend_sim.common.config.follower_config import FollowerConfig
from robot_backend_sim.common.config.motion_config import ControlConfig
from robot_backend_sim.common.follower.follower_protocol import validation_limits
from robot_contracts.action_chunks import ActionWindow, ChunkAnchor
from robot_contracts.models import RobotProfile, RunContext
from robot_contracts.stream_request import (
    REQUEST_STREAM_FIRST,
    REQUEST_STREAM_NEXT,
    StreamRequest,
    Trajectory,
)
from robot_trajectory.follower import PreparedFollowerPlan, prepare_submitted_follower
from robot_trajectory.trajectory_generation import generate_follower_plan


def generate_prepared_follower(
    anchor: ChunkAnchor,
    window: ActionWindow,
    profile: RobotProfile,
    control: ControlConfig,
    follower: FollowerConfig,
    *,
    is_final: bool,
    duration_grid_ns: int | None = None,
) -> PreparedFollowerPlan:
    context = RunContext(
        profile.robot_id, profile.digest, "backend", "test-clock",
        0, episode_id="follower-test", control_epoch=1,
    )
    request = StreamRequest(
        trajectory=Trajectory(
            context,
            "test-lease",
            anchor.stream_id,
            1,
            window.observation_id,
            anchor.start_backend_time_ns + 1_000_000_000,
            profile.joint_names,
            window.source_targets,
        ),
        kind=REQUEST_STREAM_NEXT if anchor.predecessor_seq else REQUEST_STREAM_FIRST,
        anchor=anchor,
        observation_time_ns=window.observation_time_ns,
        target_count=window.target_count,
        settle_at_end=is_final,
    )
    limits = validation_limits(control, follower, duration_grid_ns=duration_grid_ns)
    plan = generate_follower_plan(request, profile, limits)
    return prepare_submitted_follower(request, plan, profile, limits)
