"""Compose and deterministically reset the fixed-base Marvin stack scene."""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass
from types import MappingProxyType

import mujoco
import numpy as np

from robot_backend_sim.common.physics.scene_container import SceneBuildError
from robot_backend_sim.marvin_sim.assembly import (
    MarvinBinding,
    UrdfSemantics,
    add_marvin_robot,
    bind_marvin_robot,
    read_urdf_semantics,
)
from robot_backend_sim.marvin_sim.scene_base import (
    MarvinSceneBase,
    add_marvin_cameras,
    base_world_point,
    combined_yaw_quaternion,
    compile_marvin_spec,
    create_marvin_spec,
    validate_marvin_cameras,
)
from robot_backend_sim.marvin_sim.stack.environment import (
    StackBinding,
    add_stack_environment,
    bind_stack_environment,
)
from robot_backend_sim.marvin_sim.stack.scene_config import (
    MarvinStackSceneConfig,
    load_marvin_stack_scene_config,
)

__all__ = [
    "MarvinStackScene",
    "MarvinStackSceneIndices",
    "build_marvin_stack_scene",
    "load_and_build",
]

_ASSEMBLIES = {"marvin": add_marvin_robot}
_ENVIRONMENTS = {"stack": add_stack_environment}


@dataclass(frozen=True, slots=True)
class MarvinStackSceneIndices:
    robot: MarvinBinding
    environment: StackBinding
    camera_ids: Mapping[str, int]


class MarvinStackScene(MarvinSceneBase):
    """One compiled Marvin stack scene with deterministic object randomization."""

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
        model = self.model
        data = self.data
        self.reset_run_state()
        self._reset_robot_joints()

        rng = np.random.default_rng(seed)
        base = self.config.robot
        task = self.config.task
        for obj in (task.source, task.target):
            dx = (
                float(rng.uniform(-obj.randomization.xy_m, obj.randomization.xy_m))
                if obj.randomization.xy_m > 0.0
                else 0.0
            )
            dy = (
                float(rng.uniform(-obj.randomization.xy_m, obj.randomization.xy_m))
                if obj.randomization.xy_m > 0.0
                else 0.0
            )
            dyaw = (
                float(rng.uniform(-obj.randomization.yaw_deg, obj.randomization.yaw_deg))
                if obj.randomization.yaw_deg > 0.0
                else 0.0
            )
            local_x = obj.nominal_xy[0] + dx
            local_y = obj.nominal_xy[1] + dy
            local_z = task.table.top_z + obj.half_extents[2]
            world = base_world_point(
                (local_x, local_y, local_z), base.base_position, base.base_yaw_deg
            )
            quaternion = combined_yaw_quaternion(
                base.base_yaw_deg, obj.nominal_yaw_deg + dyaw
            )
            address = self.indices.environment.object_free_joint_qpos[obj.name]
            data.qpos[address : address + 7] = (*world, *quaternion)
        mujoco.mj_forward(model, data)


def build_marvin_stack_scene(config: MarvinStackSceneConfig) -> MarvinStackScene:
    """Assemble, compile and bind one Marvin stack scene."""
    if config.urdf_path is None:
        raise SceneBuildError("scene config has no resolved URDF path")
    if config.robot.assembly not in _ASSEMBLIES:
        raise SceneBuildError(f"unsupported robot assembly {config.robot.assembly!r}")
    environment = _ENVIRONMENTS.get(config.task.environment)
    if environment is None:
        raise SceneBuildError(
            f"unsupported task environment {config.task.environment!r}"
        )
    spec = create_marvin_spec(config.physics, config.robot, config.urdf_path)
    environment(
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


def load_and_build(path: str) -> MarvinStackScene:
    return build_marvin_stack_scene(load_marvin_stack_scene_config(path))
