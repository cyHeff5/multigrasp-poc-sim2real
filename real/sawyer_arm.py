"""Helper wrapper around intera_interface for Sawyer arm control."""

from typing import Dict, Optional

import rospy
import intera_interface


class SawyerArm:
    def __init__(self, limb_name: str = "right", node_name: str = "sawyer_arm_helper"):
        if not rospy.core.is_initialized():
            rospy.init_node(node_name, anonymous=True)
        self.limb = intera_interface.Limb(limb_name)

    def joint_angles(self) -> Dict[str, float]:
        for joint in self.limb.joint_angles():
            print(joint)
        return self.limb.joint_angles()

    def move_to_neutral(self, timeout: Optional[float] = None):
        if timeout is None:
            self.limb.move_to_neutral()
        else:
            self.limb.move_to_neutral(timeout=timeout)

    def move_to_joint_positions(self, angles: Dict[str, float], timeout: Optional[float] = None):
        if timeout is None:
            self.limb.move_to_joint_positions(angles)
        else:
            self.limb.move_to_joint_positions(angles, timeout=timeout)

    def set_joint_position_speed(self, speed: float):
        self.limb.set_joint_position_speed(speed)

