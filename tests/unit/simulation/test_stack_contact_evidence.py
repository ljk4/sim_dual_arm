"""Real MuJoCo contact evidence for the Marvin stack task.

The stack evaluator must count only active contacts: a positive-gap contact is
listed by MuJoCo but carries no force and must not masquerade as a finger touch
or as a grasp.  These tests build real ``MjModel``/``MjData`` pairs and drive
the same contact sampling -> observation -> evaluation chain as production.
"""

from __future__ import annotations

import sys
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

from robot_backend_sim.marvin.stack.config import load_task_config  # noqa: E402
from robot_backend_sim.marvin.stack.evaluator import (  # noqa: E402
    TaskBinding,
    _finger_contacts,
    _right_finger_source_evidence,
    capture_task_observation,
    evaluate_task,
)

TASK = load_task_config(
    ROOT / "shared/scenarios/marvin/mujoco/stack/configs/simulation/scene/marvin_stack_task.yaml"
)


def _finger_gap_model(block_x: float) -> tuple[mujoco.MjModel, mujoco.MjData]:
    """One finger-ish sphere and one free block with a positive-gap band."""
    model = mujoco.MjModel.from_xml_string(
        f"""<mujoco><option gravity="0 0 0"/><worldbody>
  <body name="finger"><geom name="finger_geom" type="sphere" size=".01"
        margin=".02" gap=".02"/></body>
  <body name="block" pos="{block_x} 0 0">
    <freejoint name="block_free"/>
    <geom name="block_geom" type="sphere" size=".01" margin=".02" gap=".02" mass="1"/>
  </body>
</worldbody></mujoco>"""
    )
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    return model, data


def _finger_block_contacts(model, data) -> tuple[int, float]:
    return _finger_contacts(
        model,
        data,
        (int(model.body("finger").id),),
        (int(model.body("block").id), int(model.body("block").id)),
    )


def test_inactive_gap_is_not_a_finger_contact():
    # Sphere surfaces 0.05 apart: inside margin + gap, outside the active
    # margin band, so MuJoCo lists the contact with efc_address == -1.
    model, data = _finger_gap_model(0.07)
    assert data.ncon == 1
    assert int(data.contact[0].efc_address) == -1
    count, force = _finger_block_contacts(model, data)
    assert count == 0
    assert force == 0.0


def test_active_contact_at_positive_distance_is_still_counted():
    # The same positive distance band is active once it lies inside the sum of
    # the geom margins; filtering by a negative distance sign would wrongly
    # discard these real constraints.
    model, data = _finger_gap_model(0.05)
    assert data.ncon == 1
    assert int(data.contact[0].efc_address) >= 0
    assert float(data.contact[0].dist) > 0.0
    count, force = _finger_block_contacts(model, data)
    assert count == 1
    assert force > 0.0


def test_active_zero_force_touch_is_still_a_contact():
    # At the active-margin edge the constraint exists but its normal force is
    # momentarily zero.  Filtering by force > 0 would hide a real touch.
    model, data = _finger_gap_model(0.06)
    assert data.ncon == 1
    assert int(data.contact[0].efc_address) >= 0
    count, force = _finger_block_contacts(model, data)
    assert count == 1
    assert force == pytest.approx(0.0)


def _two_finger_model(block_x: float) -> tuple[mujoco.MjModel, mujoco.MjData]:
    model = mujoco.MjModel.from_xml_string(
        f"""<mujoco><option gravity="0 0 0"/><worldbody>
  <body name="block" pos="{block_x} 0 0">
    <freejoint name="block_free"/>
    <geom name="block_geom" type="box" size=".03 .03 .03" mass="1"/>
  </body>
  <body name="Arm_R10_Link" pos=".033 0 0">
    <joint name="Arm_R10_Joint" type="slide" axis="1 0 0"/>
    <geom name="fl" type="sphere" size=".01" margin=".02" gap=".02"/>
  </body>
  <body name="Arm_R11_Link" pos="-.033 0 0">
    <joint name="Arm_R11_Joint" type="slide" axis="1 0 0"/>
    <geom name="fr" type="sphere" size=".01" margin=".02" gap=".02"/>
  </body>
</worldbody></mujoco>"""
    )
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    return model, data


def test_right_finger_source_evidence_ignores_the_inactive_finger():
    # One finger penetrates; the other is separated by a positive gap inside
    # the detection band.  The old count reported two fingers with min force 0.
    model, data = _two_finger_model(0.035)
    scene = SimpleNamespace(model=model)
    binding = SimpleNamespace(
        source_body_id=int(model.body("block").id),
        right_finger_body_ids=(
            int(model.body("Arm_R10_Link").id),
            int(model.body("Arm_R11_Link").id),
        ),
        right_finger_dof_addresses=(
            int(model.jnt_dofadr[model.joint("Arm_R10_Joint").id]),
            int(model.jnt_dofadr[model.joint("Arm_R11_Joint").id]),
        ),
    )
    contacts, minimum, speed = _right_finger_source_evidence(scene, data, binding)
    assert contacts == 1
    assert minimum > 0.0
    assert speed == pytest.approx(0.0)


def _stack_xml(finger_offset: float) -> str:
    return f"""<mujoco><option gravity="0 0 0"/><worldbody>
  <body name="table" pos="0 0 0">
    <geom name="table_top" type="box" size=".3 .3 .01" pos="0 0 -.01"/>
  </body>
  <body name="target" pos="0 0 .03">
    <freejoint name="target_free"/>
    <geom name="target_geom" type="box" size=".03 .03 .03" mass="1"/>
  </body>
  <body name="source" pos="0 0 .09">
    <freejoint name="source_free"/>
    <geom name="source_geom" type="box" size=".02 .02 .02" mass="1"/>
  </body>
  <body name="Arm_L10_Link" pos="0 0 5"/>
  <body name="Arm_L11_Link" pos="0 0 5"/>
  <body name="Arm_R9_Link" pos=".2 0 .2">
    <site name="Arm_R_tcp" pos="0 0 0"/>
    <site name="Arm_R_pad10_inner" pos="0 0 -0.0407"/>
    <site name="Arm_R_pad11_inner" pos="0 0 0.0407"/>
  </body>
  <body name="Arm_R10_Link" pos="{-finger_offset} 0 .09">
    <joint name="Arm_R10_Joint" type="slide" axis="1 0 0"/>
    <geom name="fl" type="sphere" size=".01" margin=".02" gap=".02"/>
  </body>
  <body name="Arm_R11_Link" pos="{finger_offset} 0 .09">
    <joint name="Arm_R11_Joint" type="slide" axis="1 0 0"/>
    <geom name="fr" type="sphere" size=".01" margin=".02" gap=".02"/>
  </body>
</worldbody></mujoco>"""


def _stack_scene(finger_offset: float):
    model = mujoco.MjModel.from_xml_string(_stack_xml(finger_offset))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    left_fingers = (
        int(model.body("Arm_L10_Link").id),
        int(model.body("Arm_L11_Link").id),
    )
    right_fingers = (
        int(model.body("Arm_R10_Link").id),
        int(model.body("Arm_R11_Link").id),
    )
    scene = SimpleNamespace(
        model=model,
        data=data,
        config=SimpleNamespace(table=SimpleNamespace(top_z=0.0)),
        indices=SimpleNamespace(
            robot=SimpleNamespace(
                pad_site_ids={
                    "right": {
                        "finger10": int(model.site("Arm_R_pad10_inner").id),
                        "finger11": int(model.site("Arm_R_pad11_inner").id),
                    }
                },
                hand_body_ids={"right": int(model.body("Arm_R9_Link").id)},
                finger_joint_qpos={
                    "right": tuple(
                        int(model.jnt_qposadr[model.joint(f"Arm_R{i}_Joint").id])
                        for i in (10, 11)
                    )
                },
            )
        ),
    )
    binding = TaskBinding(
        source_body_id=int(model.body("source").id),
        target_body_id=int(model.body("target").id),
        table_body_id=int(model.body("table").id),
        finger_body_ids=(*left_fingers, *right_fingers),
        right_finger_body_ids=right_fingers,
        left_finger_body_ids=left_fingers,
        source_qpos_address=int(model.jnt_qposadr[model.joint("source_free").id]),
        target_qpos_address=int(model.jnt_qposadr[model.joint("target_free").id]),
        source_dof_address=int(model.jnt_dofadr[model.joint("source_free").id]),
        target_dof_address=int(model.jnt_dofadr[model.joint("target_free").id]),
        right_tcp_site_id=int(model.site("Arm_R_tcp").id),
        source_half_extents_m=(0.02, 0.02, 0.02),
        target_half_extents_m=(0.03, 0.03, 0.03),
        table_top_z_m=0.0,
        right_finger_dof_addresses=(
            int(model.jnt_dofadr[model.joint("Arm_R10_Joint").id]),
            int(model.jnt_dofadr[model.joint("Arm_R11_Joint").id]),
        ),
    )
    return scene, binding


def test_inactive_gap_reaches_the_observation_without_blocking_success():
    scene, binding = _stack_scene(0.055)
    observation = capture_task_observation(scene, binding, TASK)
    evaluation = evaluate_task(observation, binding, TASK)
    assert observation.finger_contact_count == 0
    assert observation.right_finger_source_contacts == 0
    assert evaluation.condition("no_finger_contact").passed is True


def test_active_finger_contact_reaches_the_observation_and_is_rejected():
    scene, binding = _stack_scene(0.04)
    observation = capture_task_observation(scene, binding, TASK)
    evaluation = evaluate_task(observation, binding, TASK)
    assert observation.finger_contact_count == 2
    assert evaluation.condition("no_finger_contact").passed is False
