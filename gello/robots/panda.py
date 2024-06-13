import time
from typing import Dict

import numpy as np

from gello.robots.robot import Robot
import torch

MAX_OPEN = 0.09


class PandaRobot(Robot):
    """A class representing a UR robot."""

    def __init__(self, robot_ip: str = "100.97.47.74"):     
        from polymetis import GripperInterface, RobotInterface

        self.stuck_start_ts = None
        self.state = "free"

        self.print_state_ts = None
        self.counter = 0
        
        self.robot = RobotInterface(
            ip_address=robot_ip,
        )
        
        self.gripper = GripperInterface(
            ip_address=robot_ip
        )

        self.robot.set_home_pose(torch.Tensor([0, 0, 0, -1.571,  0,  1.571, 0]))
        print(self.robot.home_pose)
        self.robot.go_home()
        self.robot.start_joint_impedance()
        self.gripper.goto(width=MAX_OPEN, speed=255, force=255)
        time.sleep(1)

    def num_dofs(self) -> int:
        """Get the number of joints of the robot.

        Returns:
            int: The number of joints of the robot.
        """
        return 8

    def get_joint_state(self) -> np.ndarray:
        """Get the current state of the leader robot.

        Returns:
            T: The current state of the leader robot.
        """
        robot_joints = self.robot.get_joint_positions()
        gripper_pos = self.gripper.get_state()
        pos = np.append(robot_joints, gripper_pos.width / MAX_OPEN)
        return pos

    def command_joint_state(self, joint_state: np.ndarray) -> None:
        """Command the leader robot to a given state.

        Args:
            joint_state (np.ndarray): The state to command the leader robot to.
        """
        import torch

        self.robot.update_desired_joint_positions(torch.tensor(joint_state[:-1]))

        target_width = (MAX_OPEN * (1 - joint_state[-1]))
        current_width = self.gripper.get_state().width
        stall_timeout = 1.5
        stall_delta = -0.01  # The target to current width delta thresh to indicate gripper stuck
        free_delta = 0.01
        grasp_epsilon = 0.005

        if (self.state == "free"):
            if (target_width - current_width < stall_delta):
                if (self.stuck_start_ts == None):
                    self.stuck_start_ts = time.time()
                if (time.time() - self.stuck_start_ts > stall_timeout):
                    # Stucked for a while, it is indeed stalling
                    self.state = "stalled"
            else:
                # The current to target width delta is small, not stalled
                self.stuck_start_ts = None
                self.state = "free"
    
            if (abs(target_width - current_width) > 0.003):
                self.gripper.grasp(grasp_width=target_width, speed=1, force=1, epsilon_inner=0.001, epsilon_outer=0.001)

        if (self.state == "stalled"):
            print("Handle stall, begin grasping!")
            self.gripper.grasp(grasp_width=current_width-0.005, speed=1, force=1, epsilon_inner=0.01, epsilon_outer=0.01)
            self.stuck_start_ts = None
            self.state = "grasping"

        if (self.state == "grasping" and ((target_width - current_width) > free_delta)):
            print("Handle freeing!")
            self.gripper.grasp(grasp_width=MAX_OPEN, speed=1, force=1, epsilon_inner=0.005, epsilon_outer=0.005)
            self.state = "free"

    def get_observations(self) -> Dict[str, np.ndarray]:
        joints = self.get_joint_state()
        pos_quat = np.zeros(7)
        gripper_pos = np.array([joints[-1]])
        return {
            "joint_positions": joints,
            "joint_velocities": joints,
            "ee_pos_quat": pos_quat,
            "gripper_position": gripper_pos,
        }


def main():
    robot = PandaRobot()
    current_joints = robot.get_joint_state()
    # move a small delta 0.1 rad
    move_joints = current_joints + 0.05
    # make last joint (gripper) closed
    move_joints[-1] = 0.5
    time.sleep(1)
    m = 0.09
    robot.gripper.goto(1 * m, speed=255, force=255)
    time.sleep(1)
    robot.gripper.goto(1.05 * m, speed=255, force=255)
    time.sleep(1)
    robot.gripper.goto(1.1 * m, speed=255, force=255)
    time.sleep(1)


if __name__ == "__main__":
    main()
