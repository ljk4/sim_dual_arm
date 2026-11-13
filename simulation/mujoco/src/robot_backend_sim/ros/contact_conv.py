"""ROS conversion for one immutable contact snapshot."""

from __future__ import annotations

from typing import Any

from robot_backend_sim.physics.contact_snapshot import ContactSnapshot

__all__ = ["contact_snapshot_message"]


def contact_snapshot_message(snapshot: ContactSnapshot, context: Any) -> Any:
    """Copy one snapshot into the simulation diagnostic message."""
    from robot_ros_support.conversions import context_message, stamp
    from sim_api_msgs.msg import ContactSnapshot as ContactSnapshotMessage

    return ContactSnapshotMessage(
        context=context_message(context),
        sample_time=stamp(snapshot.sim_time_ns),
        capture_seq=snapshot.capture_seq,
        object_names=[item.name for item in snapshot.objects],
        object_center=[value for item in snapshot.objects for value in item.center],
        object_quaternion=[
            value for item in snapshot.objects for value in item.quaternion_wxyz
        ],
        object_linear_velocity=[
            value for item in snapshot.objects for value in item.linear_velocity
        ],
        object_angular_velocity=[
            value for item in snapshot.objects for value in item.angular_velocity
        ],
        contact_side=[item.side for item in snapshot.finger_contacts],
        contact_finger=[item.finger_index for item in snapshot.finger_contacts],
        contact_object=[item.object_name for item in snapshot.finger_contacts],
        contact_normal_force=[item.normal_force_n for item in snapshot.finger_contacts],
        contact_count=[item.contact_count for item in snapshot.finger_contacts],
        support_object=[item.object_name for item in snapshot.support_contacts],
        support_body=[item.body_name for item in snapshot.support_contacts],
        support_normal_force=[item.normal_force_n for item in snapshot.support_contacts],
        support_contact_count=[item.contact_count for item in snapshot.support_contacts],
        gripper_sides=[item.side for item in snapshot.grippers],
        gripper_opening=[item.opening_m for item in snapshot.grippers],
        tcp_position=[value for item in snapshot.grippers for value in item.tcp_position],
        tcp_quaternion=[
            value for item in snapshot.grippers for value in item.tcp_quaternion_wxyz
        ],
    )
