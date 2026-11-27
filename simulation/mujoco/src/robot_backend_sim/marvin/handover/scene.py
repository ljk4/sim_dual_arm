"""Compose the fixed-base Marvin handover scene.

Robot assembly, task environment and the generic MuJoCo compile/reset
container are composed here by explicit configuration selection.  The task
name does not choose the robot, and the robot assembly does not know the
handover stages.
"""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass
from types import MappingProxyType

import mujoco

from robot_backend_sim.common.physics.scene_container import SceneBuildError
from robot_backend_sim.marvin.assembly import (
    MarvinBinding,
    UrdfSemantics,
    bind_marvin_robot,
    read_urdf_semantics,
)
from robot_backend_sim.marvin.handover.environment import (
    HandoverBinding,
    add_handover_environment,
    bind_handover_environment,
)
from robot_backend_sim.marvin.handover.scene_config import MarvinSceneConfig
from robot_backend_sim.marvin.scene_base import (
    MarvinSceneBase,
    add_marvin_cameras,
    base_world_point,
    combined_yaw_quaternion,
    compile_marvin_spec,
    create_marvin_spec,
    validate_marvin_cameras,
)

__all__ = [
    "MarvinScene",
    "MarvinSceneIndices",
    "build_marvin_scene",
]


@dataclass(frozen=True, slots=True)
class MarvinSceneIndices:
    robot: MarvinBinding
    environment: HandoverBinding
    camera_ids: Mapping[str, int]


class MarvinScene(MarvinSceneBase):
    """One compiled Marvin handover scene with deterministic reset."""

    @property
    def contact_object_names(self) -> tuple[str, ...]:
        return (self.config.task.block.name,)

    @property
    def object_free_joint_qpos(self) -> Mapping[str, int]:
        return {
            self.config.task.block.name: self.indices.environment.block_free_joint_qpos
        }

    @property
    def support_body_names(self) -> tuple[str, ...]:
        return ("table", self.config.task.box.name)

    def reset(self, seed: int) -> None:
        if isinstance(seed, bool) or not isinstance(seed, int):
            raise TypeError("seed must be an integer")
        if seed < 0:
            raise ValueError("seed must be a non-negative integer")
        model = self.model
        data = self.data
        self.reset_run_state()
        self._reset_robot_joints()

        block = self.config.task.block
        address = self.indices.environment.block_free_joint_qpos
        base = self.config.robot
        world = base_world_point(
            (
                block.initial_xy[0],
                block.initial_xy[1],
                self.config.task.table.top_z + block.half_extents[2],
            ),
            base.base_position,
            base.base_yaw_deg,
        )
        quaternion = combined_yaw_quaternion(base.base_yaw_deg, block.initial_yaw_deg)
        data.qpos[address : address + 7] = (*world, *quaternion)
        self._apply_domain_randomization(seed)
        mujoco.mj_forward(model, data)


def build_marvin_scene(config: MarvinSceneConfig) -> MarvinScene:
    """Assemble, compile and bind one Marvin handover scene."""
    if config.urdf_path is None:
        raise SceneBuildError("scene config has no resolved URDF path")
    if config.robot.assembly != "marvin":
        raise SceneBuildError(f"unsupported robot assembly {config.robot.assembly!r}")
    if config.task.environment != "handover":
        raise SceneBuildError(f"unsupported task environment {config.task.environment!r}")
    spec = create_marvin_spec(config.physics, config.robot, config.urdf_path)
    add_handover_environment(
        spec,
        config.task,
        config.physics.object_friction,
        config.robot.base_position,
        config.robot.base_yaw_deg,
    )
    spec.geom(f"{config.task.block.name}_geom").condim = config.physics.object_contact_dim
    add_marvin_cameras(spec, config.cameras)
    model, data = compile_marvin_spec(spec, config.cameras, modelname="marvin_handover")

    semantics: UrdfSemantics = read_urdf_semantics(config.urdf_path)
    robot_binding = bind_marvin_robot(model, config.robot, semantics)
    environment_binding = bind_handover_environment(model, config.task)
    indices = MarvinSceneIndices(
        robot=robot_binding,
        environment=environment_binding,
        camera_ids=MappingProxyType(validate_marvin_cameras(model, config.cameras)),
    )
    scene = MarvinScene(model, data, config, indices)
    scene.reset(0)
    return scene
