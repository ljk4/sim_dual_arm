"""Compose and deterministically reset the fixed-base Marvin stack scene."""

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
from robot_backend_sim.marvin.scene_base import (
    MarvinSceneBase,
    add_marvin_cameras,
    compile_marvin_spec,
    create_marvin_spec,
    validate_marvin_cameras,
)
from robot_backend_sim.marvin.stack.environment import (
    StackBinding,
    add_stack_environment,
    bind_stack_environment,
)
from robot_backend_sim.marvin.stack.scene_config import MarvinStackSceneConfig

__all__ = [
    "MarvinStackScene",
    "MarvinStackSceneIndices",
    "build_marvin_stack_scene",
]


@dataclass(frozen=True, slots=True)
class MarvinStackSceneIndices:
    robot: MarvinBinding
    environment: StackBinding
    camera_ids: Mapping[str, int]


class MarvinStackScene(MarvinSceneBase):
    """One compiled Marvin stack scene with fixed initial object poses."""

    @property
    def contact_object_names(self) -> tuple[str, ...]:
        return (self.config.task.source.name, self.config.task.target.name)

    @property
    def object_free_joint_qpos(self) -> Mapping[str, int]:
        return self.indices.environment.object_free_joint_qpos

    @property
    def support_body_names(self) -> tuple[str, ...]:
        return ("table", self.config.task.target.name)

    def reset(self, seed: int) -> None:
        if isinstance(seed, bool) or not isinstance(seed, int):
            raise TypeError("seed must be an integer")
        if seed < 0:
            raise ValueError("seed must be a non-negative integer")
        # The free joints' compiled rest poses are the configured nominal poses.
        self.reset_run_state()
        self._reset_robot_joints()
        self._apply_domain_randomization(seed)
        mujoco.mj_forward(self.model, self.data)


def build_marvin_stack_scene(config: MarvinStackSceneConfig) -> MarvinStackScene:
    """Assemble, compile and bind one Marvin stack scene."""
    if config.urdf_path is None:
        raise SceneBuildError("scene config has no resolved URDF path")
    if config.robot.assembly != "marvin":
        raise SceneBuildError(f"unsupported robot assembly {config.robot.assembly!r}")
    if config.task.environment != "stack":
        raise SceneBuildError(f"unsupported task environment {config.task.environment!r}")
    spec = create_marvin_spec(config.physics, config.robot, config.urdf_path)
    add_stack_environment(
        spec,
        config.task,
        config.physics.object_friction,
        config.robot.base_position,
        config.robot.base_yaw_deg,
    )
    for obj in (config.task.source, config.task.target):
        spec.geom(f"{obj.name}_geom").condim = config.physics.object_contact_dim
    add_marvin_cameras(spec, config.cameras)
    model, data = compile_marvin_spec(spec, config.cameras, modelname="marvin_stack")

    semantics: UrdfSemantics = read_urdf_semantics(config.urdf_path)
    robot_binding = bind_marvin_robot(model, config.robot, semantics)
    environment_binding = bind_stack_environment(model, config.task)
    indices = MarvinStackSceneIndices(
        robot=robot_binding,
        environment=environment_binding,
        camera_ids=MappingProxyType(validate_marvin_cameras(model, config.cameras)),
    )
    scene = MarvinStackScene(model, data, config, indices)
    scene.reset(0)
    return scene
