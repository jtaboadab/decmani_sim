"""Configuración del robot WX250 para pymoveit2."""

from typing import List

MOVE_GROUP_ARM: str = "arm"
MOVE_GROUP_GRIPPER: str = "gripper"

OPEN_GRIPPER_JOINT_POSITIONS: List[float] = [0.037]
CLOSED_GRIPPER_JOINT_POSITIONS: List[float] = [0.005]


def joint_names(prefix: str = "") -> List[str]:
    return [
        prefix + "waist",
        prefix + "shoulder",
        prefix + "elbow",
        prefix + "wrist_angle",
        prefix + "wrist_rotate",
    ]


def base_link_name(prefix: str = "wx250/") -> str:
    return prefix + "base_link"


def end_effector_name(prefix: str = "wx250/") -> str:
    return prefix + "gripper_link"


def gripper_joint_names(prefix: str = "") -> List[str]:
    return [
        prefix + "left_finger",
    ]