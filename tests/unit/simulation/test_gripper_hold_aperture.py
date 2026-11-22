"""Real MuJoCo check of the premise retiming relies on.

A position-controlled parallel gripper that closes onto a block cannot reach the
commanded aperture: the measured opening stays at the block width while both
pads keep real, active contact.  The bounded follower therefore inherits the
*reference* aperture, which is narrower than the measured one, and a following
release command can be infeasible from that inherited value.  This test does not
run the follower; it proves the physical contact state with a real MjModel.
"""

from __future__ import annotations

import mujoco
import numpy as np

_XML = """
<mujoco>
 <option timestep="0.002" gravity="0 0 0" integrator="implicitfast"/>
 <worldbody>
  <body name="block" pos="0 0 0">
    <freejoint name="block_free"/>
    <geom name="block_geom" type="box" size="0.02 0.02 0.02" mass="0.2"
          friction="2 0.1 0.1"/>
  </body>
  <body name="left_finger" pos="-0.035 0 0">
    <joint name="left_slide" type="slide" axis="1 0 0" range="0 0.03"
           damping="1" armature="0.01"/>
    <geom name="left_pad" type="box" size="0.005 0.02 0.02" friction="2 0.1 0.1"/>
  </body>
  <body name="right_finger" pos="0.035 0 0">
    <joint name="right_slide" type="slide" axis="1 0 0" range="-0.03 0"
           damping="1" armature="0.01"/>
    <geom name="right_pad" type="box" size="0.005 0.02 0.02" friction="2 0.1 0.1"/>
  </body>
 </worldbody>
 <actuator>
   <position name="left_act" joint="left_slide" kp="300" kv="25" ctrlrange="0 0.03"/>
   <position name="right_act" joint="right_slide" kp="300" kv="25" ctrlrange="-0.03 0"/>
 </actuator>
</mujoco>
"""


def _run_closed(reference_width: float) -> tuple[float, float, list[tuple[str, str, float]]]:
    model = mujoco.MjModel.from_xml_string(_XML)
    data = mujoco.MjData(model)
    mujoco.mj_resetData(model, data)
    # Opening = 0.07 + right_slide - left_slide.  Split the target per finger.
    offset = (0.07 - reference_width) / 2.0
    data.ctrl[model.actuator("left_act").id] = offset
    data.ctrl[model.actuator("right_act").id] = -offset
    for _ in range(1500):
        mujoco.mj_step(model, data)
    mujoco.mj_forward(model, data)
    left = float(data.qpos[model.joint("left_slide").qposadr[0]])
    right = float(data.qpos[model.joint("right_slide").qposadr[0]])
    measured = 0.07 + right - left
    contacts = []
    force = np.zeros(6)
    for index in range(data.ncon):
        contact = data.contact[index]
        names = tuple(sorted((model.geom(contact.geom1).name, model.geom(contact.geom2).name)))
        if "block_geom" not in names:
            continue
        assert contact.efc_address >= 0
        mujoco.mj_contactForce(model, data, index, force)
        contacts.append((names[0], names[1], float(force[0])))
    block_x = float(data.qpos[model.joint("block_free").qposadr[0]])
    return measured, block_x, contacts


def test_measured_aperture_stays_above_a_closed_reference_with_real_contact():
    reference_width = 0.02
    measured, block_x, contacts = _run_closed(reference_width)
    assert measured > reference_width + 0.01
    assert abs(block_x) < 0.005
    assert any(left == "block_geom" and right == "left_pad" for left, right, _ in contacts)
    assert any(left == "block_geom" and right == "right_pad" for left, right, _ in contacts)
    assert all(force > 0.0 for _, _, force in contacts)
