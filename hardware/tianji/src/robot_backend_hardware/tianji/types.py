from dataclasses import dataclass
from enum import IntEnum


class MotionMode(IntEnum):
    """Motion execution mode for actuators."""

    TRAJECTORY = 0
    ONLINE_TRACKING = 1


class CommandSpace(IntEnum):
    """Target space used by the active motion mode."""

    UNSPECIFIED = 0
    CART = 1
    JOINT = 2


@dataclass(frozen=True)
class MotionModeState:
    """Device-local motion mode and target space."""

    mode: MotionMode
    command_space: CommandSpace


class StateKey:
    LEFTARM_STATE_JOINT_POS   = "leftarm_state_joint_pos"
    LEFTARM_STATE_CART_POS    = "leftarm_state_cart_pos"
    LEFTARM_STATE_PSI         = "leftarm_state_psi"
    LEFTARM_STATE_JOINT_TORQUE = "leftarm_state_joint_torque"
    LEFTARM_GRIPPER_STATE_POS = "leftarm_gripper_state_pos"

    RIGHTARM_STATE_JOINT_POS   = "rightarm_state_joint_pos"
    RIGHTARM_STATE_CART_POS    = "rightarm_state_cart_pos"
    RIGHTARM_STATE_PSI         = "rightarm_state_psi"
    RIGHTARM_STATE_JOINT_TORQUE = "rightarm_state_joint_torque"
    RIGHTARM_GRIPPER_STATE_POS = "rightarm_gripper_state_pos"

    CAM_HIGH        = "cam_high"
    CAM_LEFT_WRIST  = "cam_left_wrist"
    CAM_RIGHT_WRIST = "cam_right_wrist"


class CmdKey:
    LEFTARM_CMD_CART_POS    = "leftarm_cmd_cart_pos"
    LEFTARM_CMD_JOINT_POS   = "leftarm_cmd_joint_pos"
    LEFTARM_GRIPPER_CMD_POS = "leftarm_gripper_cmd_pos"

    RIGHTARM_CMD_CART_POS    = "rightarm_cmd_cart_pos"
    RIGHTARM_CMD_JOINT_POS   = "rightarm_cmd_joint_pos"
    RIGHTARM_GRIPPER_CMD_POS = "rightarm_gripper_cmd_pos"


    # Online tracking targets (used in ONLINE_TRACKING mode)
    LEFTARM_TRACKING_TARGET  = "leftarm_tracking_target"
    RIGHTARM_TRACKING_TARGET = "rightarm_tracking_target"
