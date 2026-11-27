"""Real MuJoCo contact evidence for the handover task.

These tests build a minimal but real ``MjModel``/``MjData`` with the same body,
geom and finger roles the handover adapter binds.  They exercise the measured
chain contact sampling -> task observation -> evaluation; synthetic state
machine cases live in ``test_handover_task.py``.
"""

from __future__ import annotations

import sys
from dataclasses import replace
from pathlib import Path
from types import SimpleNamespace

import mujoco
import pytest

ROOT = Path(__file__).resolve().parents[3]
SIM_SRC = ROOT / "simulation/mujoco/src"
CONTRACTS = ROOT / "shared/common/python/contracts/src"
for source in (SIM_SRC, CONTRACTS):
    if str(source) not in sys.path:
        sys.path.insert(0, str(source))

from robot_backend_sim.marvin.handover.evaluator import (  # noqa: E402
    HandoverTaskRuntime,
    evaluation_conditions,
)
from robot_backend_sim.marvin.handover.observation import (  # noqa: E402
    HandoverBinding,
    capture_handover_observation,
)
from robot_backend_sim.marvin.handover.plan import HandoverLayout  # noqa: E402
from robot_backend_sim.marvin.handover.task_config import (  # noqa: E402
    load_handover_task_config,
)
from robot_contracts.models import RunContext  # noqa: E402

SCENARIO = ROOT / "shared/scenarios/marvin/mujoco/handover"
TASK = load_handover_task_config(
    SCENARIO / "configs/simulation/scene/marvin_handover_task.yaml"
)

LAYOUT = HandoverLayout(
    base_position=(0.0, 0.0, 0.0),
    base_yaw_deg=0.0,
    table_top_z=0.0,
    block_name="block",
    block_half_extents=(0.03, 0.03, 0.03),
    block_initial_center=(0.0, 0.0, 0.1),
    block_initial_yaw_rad=0.0,
    box_name="box",
    box_center=(0.3, 0.0, 0.0),
    box_inner_xy=(0.18, 0.18),
    box_inner_bottom_z=0.01,
    box_rim_z=0.085,
    box_yaw_rad=0.0,
)


def _xml(
    *,
    left_hand: tuple[float, float, float] = (0.0, 0.0, 0.3),
    right_hand: tuple[float, float, float] = (0.0, 0.0, 0.3),
    left_pad_offset: float = 0.033,
    right_pad_offset: float = 0.033,
    pad_margin: float = 0.0,
    block_first: bool = False,
) -> str:
    """A real hand/box/table model with the handover body and geom names."""
    environment = """<body name="table" pos="0 0 0">
    <geom name="table_top" type="box" size=".3 .3 .01" pos="0 0 -.01"/>
  </body>
  <body name="box" pos=".3 0 0">
    <geom name="box_bottom" type="box" size=".1 .1 .005" pos="0 0 .005"/>
    <geom name="box_wall_x_pos" type="box" size=".005 .1 .04" pos=".095 0 .045"/>
    <geom name="box_wall_x_neg" type="box" size=".005 .1 .04" pos="-.095 0 .045"/>
    <geom name="box_wall_y_pos" type="box" size=".09 .005 .04" pos="0 .095 .045"/>
    <geom name="box_wall_y_neg" type="box" size=".09 .005 .04" pos="0 -.095 .045"/>
  </body>"""
    block = """<body name="block" pos="0 0 .3">
    <freejoint name="block_free"/>
    <geom name="block_geom" type="box" size=".03 .03 .03" mass="1"/>
  </body>"""
    hands = f"""<body name="Arm_L_hand" pos="{left_hand[0]} {left_hand[1]} {left_hand[2]}">
    <body name="Arm_L_finger1" pos="{left_pad_offset} 0 0">
      <geom name="Arm_L10_geom" type="sphere" size=".01"
            margin="{pad_margin}" gap="{pad_margin}"/>
      <site name="Arm_L_pad1_inner"/>
    </body>
    <body name="Arm_L_finger2" pos="{-left_pad_offset} 0 0">
      <geom name="Arm_L11_geom" type="sphere" size=".01"
            margin="{pad_margin}" gap="{pad_margin}"/>
      <site name="Arm_L_pad2_inner"/>
    </body>
    <site name="Arm_L_tcp"/>
  </body>
  <body name="Arm_R_hand" pos="{right_hand[0]} {right_hand[1]} {right_hand[2]}">
    <body name="Arm_R_finger1" pos="0 {right_pad_offset} 0">
      <geom name="Arm_R10_geom" type="sphere" size=".01"
            margin="{pad_margin}" gap="{pad_margin}"/>
      <site name="Arm_R_pad1_inner"/>
    </body>
    <body name="Arm_R_finger2" pos="0 {-right_pad_offset} 0">
      <geom name="Arm_R11_geom" type="sphere" size=".01"
            margin="{pad_margin}" gap="{pad_margin}"/>
      <site name="Arm_R_pad2_inner"/>
    </body>
    <site name="Arm_R_tcp"/>
  </body>"""
    order = block + environment if block_first else environment + block
    return f'<mujoco><option gravity="0 0 0"/><worldbody>{order}{hands}</worldbody></mujoco>'


class _Scene:
    """The minimal attributes the handover capture reads from a compiled scene."""

    def __init__(self, xml: str) -> None:
        self.model = mujoco.MjModel.from_xml_string(xml)
        self.data = mujoco.MjData(self.model)
        model = self.model
        self.body_names = tuple(
            str(model.body(index).name) or f"body[{index}]" for index in range(model.nbody)
        )
        self.geom_names = tuple(
            str(model.geom(index).name) or f"geom[{index}]" for index in range(model.ngeom)
        )
        self.block_qpos = int(model.jnt_qposadr[model.joint("block_free").id])
        self.block_dof = int(model.jnt_dofadr[model.joint("block_free").id])
        self.object_free_joint_qpos = {"block": self.block_qpos}
        self.contact_object_names = ("block",)
        self.robot = SimpleNamespace(
            pad_site_ids={
                "left": {
                    "finger1": int(model.site("Arm_L_pad1_inner").id),
                    "finger2": int(model.site("Arm_L_pad2_inner").id),
                },
                "right": {
                    "finger1": int(model.site("Arm_R_pad1_inner").id),
                    "finger2": int(model.site("Arm_R_pad2_inner").id),
                },
            },
            hand_body_ids={
                "left": int(model.body("Arm_L_hand").id),
                "right": int(model.body("Arm_R_hand").id),
            },
            arm_joint_qpos={"left": tuple(range(7)), "right": tuple(range(7))},
            arm_joint_dof={"left": tuple(range(6)), "right": tuple(range(6))},
        )
        self.robot_geometry = SimpleNamespace(
            finger_bodies={
                "left": (
                    ("Arm_L_finger1", "Arm_L_finger1_Joint"),
                    ("Arm_L_finger2", "Arm_L_finger2_Joint"),
                ),
                "right": (
                    ("Arm_R_finger1", "Arm_R_finger1_Joint"),
                    ("Arm_R_finger2", "Arm_R_finger2_Joint"),
                ),
            },
            chains=(
                SimpleNamespace(side="left", tcp_site="Arm_L_tcp"),
                SimpleNamespace(side="right", tcp_site="Arm_R_tcp"),
            ),
        )
        self.indices = SimpleNamespace(robot=self.robot)

    def set_block(self, center: tuple[float, float, float]) -> None:
        self.data.qpos[self.block_qpos : self.block_qpos + 7] = (*center, 1.0, 0.0, 0.0, 0.0)
        self.data.qvel[:] = 0.0
        mujoco.mj_forward(self.model, self.data)

    def binding(self) -> HandoverBinding:
        model = self.model
        robot_bodies = frozenset(
            index
            for index in range(model.nbody)
            if str(model.body(index).name).startswith("Arm_")
        )
        return HandoverBinding(
            block_name="block",
            block_body_id=int(model.body("block").id),
            block_geom_name="block_geom",
            block_qpos_address=self.block_qpos,
            block_dof_address=self.block_dof,
            table_body_id=int(model.body("table").id),
            box_body_id=int(model.body("box").id),
            support_body_names=("table", "box"),
            box_bottom_geom_names=("box_bottom",),
            left_finger_dof_addresses=(0, 1),
            right_finger_dof_addresses=(2, 3),
            left_body_ids=frozenset(
                int(model.body(name).id)
                for name in ("Arm_L_hand", "Arm_L_finger1", "Arm_L_finger2")
            ),
            right_body_ids=frozenset(
                int(model.body(name).id)
                for name in ("Arm_R_hand", "Arm_R_finger1", "Arm_R_finger2")
            ),
            robot_body_ids=robot_bodies,
            table_top_z_m=0.0,
            block_half_extents_m=(0.03, 0.03, 0.03),
            box_inner_bottom_z_m=0.01,
            box_center_xy=(0.3, 0.0),
            box_inner_xy=(0.18, 0.18),
            box_yaw_rad=0.0,
            block_initial_center=(0.0, 0.0, 0.1),
        )


def _conditions(observation, binding, *, sequence_complete: bool = True):
    return {
        condition.name: condition
        for condition in evaluation_conditions(
            observation,
            binding,
            TASK,
            LAYOUT,
            position_reference=None,
            rotation_reference=None,
            sequence_complete=sequence_complete,
        )
    }


def test_real_double_finger_hold_is_measured_from_contacts():
    scene = _Scene(_xml())
    scene.set_block((0.0, 0.0, 0.3))
    binding = scene.binding()
    observation = capture_handover_observation(scene, binding)
    minimum = TASK.planning.diagnostics.contact_force_min_n
    assert observation.holding("left", minimum) is True
    assert observation.holding("right", minimum) is True
    assert [contact.finger_index for contact in observation.left_contacts] == [1, 2]
    assert all(
        contact.normal_force_n >= minimum for contact in observation.left_contacts
    )
    assert observation.support_contacts == ()


def test_inactive_gap_contacts_do_not_block_release():
    # Both left pads sit in the positive-gap band: the contact list retains an
    # inactive pair but the fingers do not physically touch the block.
    scene = _Scene(_xml(left_pad_offset=0.075, pad_margin=0.02))
    scene.set_block((0.0, 0.0, 0.3))
    binding = scene.binding()
    observation = capture_handover_observation(scene, binding)
    assert observation.left_contacts == ()
    conditions = _conditions(observation, binding)
    assert conditions["left_released"].passed is True


def test_real_box_bottom_support_only_comes_from_the_bottom_geom():
    scene = _Scene(_xml())
    scene.set_block((0.3, 0.0, 0.039))
    binding = scene.binding()
    observation = capture_handover_observation(scene, binding)
    bottom = observation.support_upward_force(("box_bottom",))
    assert bottom >= TASK.success.min_box_support_normal_force_n
    assert observation.support_force("box") == pytest.approx(bottom)
    conditions = _conditions(observation, binding)
    assert conditions["box_bottom_support"].passed is True
    assert conditions["not_on_table"].passed is True
    assert conditions["block_contained"].passed is True


def test_box_bottom_support_resolves_the_geom_order_and_direction():
    scene = _Scene(_xml(block_first=True))
    scene.set_block((0.3, 0.0, 0.039))
    binding = scene.binding()
    block_geom = int(scene.model.geom("block_geom").id)
    assert any(
        int(scene.data.contact[index].geom1) == block_geom
        for index in range(int(scene.data.ncon))
    )
    observation = capture_handover_observation(scene, binding)
    bottom = observation.support_upward_force(("box_bottom",))
    assert bottom >= TASK.success.min_box_support_normal_force_n
    conditions = _conditions(observation, binding)
    assert conditions["box_bottom_support"].passed is True


def test_wall_only_support_cannot_replace_the_box_bottom():
    scene = _Scene(_xml())
    scene.set_block((0.361, 0.0, 0.1))
    binding = scene.binding()
    observation = capture_handover_observation(scene, binding)
    assert observation.support_upward_force(("box_bottom",)) == pytest.approx(0.0)
    assert observation.support_force("box") > 0.0
    conditions = _conditions(observation, binding)
    assert conditions["box_bottom_support"].passed is False
    assert conditions["not_on_table"].passed is True


def test_table_support_is_not_box_bottom_support():
    scene = _Scene(_xml())
    scene.set_block((-0.2, 0.0, 0.029))
    binding = scene.binding()
    observation = capture_handover_observation(scene, binding)
    assert observation.support_force("table") > 0.0
    assert observation.support_upward_force(("box_bottom",)) == pytest.approx(0.0)
    conditions = _conditions(observation, binding)
    assert conditions["box_bottom_support"].passed is False
    assert conditions["not_on_table"].passed is False


def test_captured_contacts_drive_the_policy_sequence_gate():
    # The right hand is moved away so the captured evidence is a left-sole hold.
    scene = _Scene(_xml(right_hand=(0.0, 0.3, 0.3)))
    scene.set_block((0.0, 0.0, 0.3))
    binding = scene.binding()
    observation = capture_handover_observation(scene, binding)
    minimum = TASK.planning.diagnostics.contact_force_min_n
    assert observation.holding("left", minimum) is True
    assert observation.holding("right", minimum) is False
    runtime = HandoverTaskRuntime(TASK, binding, LAYOUT)
    context = RunContext("marvin", "profile", "backend", "sim", 0, "episode", 0)
    runtime.commit_prepare(
        request_id="prepare",
        content_digest="content",
        context=context,
        plan=SimpleNamespace(
            steps=(),
            plan_digest="0" * 64,
            duration_ns=1_000_000_000,
            open_total_m=0.090,
        ),
        plan_digest="0" * 64,
        plan_duration_ns=1_000_000_000,
        step_count=0,
    )
    runtime.start(
        request_id="start",
        content_digest="start-content",
        context=context,
        plan_digest="0" * 64,
        task_epoch=runtime.task_epoch,
        sim_time_ns=0,
        wall_ns=0,
    )
    state = None
    for step in range(1, 7):
        time_ns = step * 100_000_000
        state = runtime.tick(replace(observation, sim_time_ns=time_ns), time_ns)
    assert state is not None and state.phase == "RUNNING"
    sequence = state.metrics["handover_sequence"]
    assert sequence["left_airborne_stable"] is True
    assert sequence["complete"] is False
