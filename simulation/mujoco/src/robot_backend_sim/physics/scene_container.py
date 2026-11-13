"""Scene compilation and run-state reset shared by simulation scenes.

Robot assembly and task-environment modules add their own bodies, geoms,
joints and actuators to an ``MjSpec``.  This container only turns that spec
into a model/data pair and provides the deterministic episode-boundary reset
of every writable run-state field.  It knows nothing about a specific robot or
task.
"""

from __future__ import annotations

from collections.abc import Mapping, Sequence

import mujoco

from robot_backend_sim.physics.robot_geometry import RobotGeometry


class SceneBuildError(RuntimeError):
    """The assembled MuJoCo model is not usable."""


class CompiledScene:
    """One compiled model and its writable data."""

    def __init__(self, model: mujoco.MjModel, data: mujoco.MjData) -> None:
        self.model = model
        self.data = data
        # The compiled model is immutable, so resolve the body-name table once
        # instead of rebuilding it on every 500 Hz contact-snapshot capture.
        self.body_names: tuple[str, ...] = tuple(
            str(model.body(index).name) or f"body[{index}]"
            for index in range(int(model.nbody))
        )

    @property
    def contact_object_names(self) -> tuple[str, ...]:
        """Names of the free bodies that the task environment exposes to contacts.

        Robot and task assemblies provide this identity; the generic compile and
        reset container does not guess it from model adjacency.
        """
        raise NotImplementedError("the scene assembly must declare its contact objects")

    @property
    def robot_geometry(self) -> RobotGeometry:
        """Named robot chains, body groups and finger bindings for planning."""
        raise NotImplementedError("the robot assembly must declare its geometry")

    @property
    def object_free_joint_qpos(self) -> Mapping[str, int]:
        """Free-joint qpos address of every configured contact object."""
        raise NotImplementedError("the task environment must declare object addresses")

    @property
    def support_body_names(self) -> tuple[str, ...]:
        """Non-object bodies that can support a configured contact object."""
        raise NotImplementedError("the task environment must declare support bodies")

    def reset_run_state(self) -> None:
        """Clear all writable run state before a scene writes its initial pose.

        ``mj_resetData`` restores the compiled defaults; the explicit clear
        also covers fields that a caller may have written during a run and
        keeps the reset independent of integrator-specific warm-start caches.
        """
        model = self.model
        data = self.data
        mujoco.mj_resetData(model, data)
        data.time = 0.0
        data.qpos[:] = model.qpos0
        data.qvel[:] = 0.0
        data.act[:] = 0.0
        data.ctrl[:] = 0.0
        data.qfrc_applied[:] = 0.0
        data.xfrc_applied[:] = 0.0
        data.qacc_warmstart[:] = 0.0


def compile_spec(
    spec: mujoco.MjSpec,
    cameras: Sequence[object],
    *,
    modelname: str | None = None,
) -> tuple[mujoco.MjModel, mujoco.MjData]:
    """Compile one assembled spec and size the offscreen framebuffer.

    Camera declarations are plain objects with ``width`` and ``height`` fields;
    the scene-specific assembly validates name, parent and lens after compile.
    """
    if modelname is not None:
        spec.modelname = modelname
    if cameras:
        spec.visual.global_.offwidth = max(int(camera.width) for camera in cameras)
        spec.visual.global_.offheight = max(int(camera.height) for camera in cameras)
    try:
        model = spec.compile()
    except Exception as error:
        raise SceneBuildError(f"MuJoCo scene compilation failed: {error}") from error
    return model, mujoco.MjData(model)
