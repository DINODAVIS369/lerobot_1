#!/usr/bin/env python3
"""
Move the SO-ARM-100 robot arm using FollowJointTrajectory action client.
Run while the MoveIt launch file is running.

Usage:
    source install/setup.bash
    python3 src/lerobot_moveit/scripts/move_arm.py

Adjust TARGET_ARM_POSITIONS and TARGET_GRIPPER_POSITION to safe positions.
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration as RclpyDuration
from control_msgs.action import FollowJointTrajectory
from control_msgs.msg import JointTolerance
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration


# ── CONFIGURATION ────────────────────────────────────────────────────────────
ARM_JOINTS    = ["joint1", "joint2", "joint3", "joint4", "joint5"]
GRIPPER_JOINTS = ["joint6"]

# Target joint positions (radians). Adjust for your robot!
TARGET_ARM_POSITIONS     = [0.3, -0.4, 0.4, 0.0, 0.2]   # joint1..5
TARGET_GRIPPER_POSITION  = [0.5]                           # joint6

TRAVEL_TIME_SEC = 4    # seconds for the arm to reach the target
# ─────────────────────────────────────────────────────────────────────────────


def make_tolerances(joint_names, position_tol=0.2, velocity_tol=0.5, accel_tol=0.5):
    """Build a list of JointTolerance (generous tolerances so velocity-limited
       servo controllers always accept a waypoint as 'reached')."""
    tolerances = []
    for name in joint_names:
        t = JointTolerance()
        t.name = name
        t.position = position_tol
        t.velocity = velocity_tol
        t.acceleration = accel_tol
        tolerances.append(t)
    return tolerances


class ArmMover(Node):
    def __init__(self):
        super().__init__("arm_mover")
        self._arm_client = ActionClient(
            self, FollowJointTrajectory,
            "/arm_controller/follow_joint_trajectory")
        self._gripper_client = ActionClient(
            self, FollowJointTrajectory,
            "/gripper_controller/follow_joint_trajectory")

    def _send_goal(self, client, joint_names, positions, travel_sec):
        self.get_logger().info(f"Waiting for action server: {client._action_name}")
        client.wait_for_server()

        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = joint_names

        point = JointTrajectoryPoint()
        point.positions = list(positions)
        point.time_from_start = Duration(sec=travel_sec)
        goal.trajectory.points = [point]

        # Generous tolerances so the controller accepts the move as done
        goal.goal_tolerance          = make_tolerances(joint_names, position_tol=0.3)
        goal.path_tolerance          = []  # Disable path tolerance checking
        goal.goal_time_tolerance     = Duration(sec=15)   # allow 15 extra seconds

        def dummy_feedback_cb(feedback_msg):
            pass

        self.get_logger().info(f"Sending goal: {dict(zip(joint_names, positions))}")
        future = client.send_goal_async(goal, feedback_callback=dummy_feedback_cb)
        rclpy.spin_until_future_complete(self, future)
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().error("Goal REJECTED by controller!")
            return False

        self.get_logger().info("Goal accepted. Waiting for result...")
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result()

        if result.status == 4:   # SUCCEEDED
            self.get_logger().info("Motion SUCCEEDED!")
            return True
        else:
            self.get_logger().warn(f"Motion finished with status: {result.status}  "
                                   f"error_code: {result.result.error_code}")
            return False

    def move_arm(self):
        return self._send_goal(self._arm_client, ARM_JOINTS,
                               TARGET_ARM_POSITIONS, TRAVEL_TIME_SEC)

    def move_gripper(self):
        return self._send_goal(self._gripper_client, GRIPPER_JOINTS,
                               TARGET_GRIPPER_POSITION, 2)


def main(args=None):
    rclpy.init(args=args)
    node = ArmMover()
    node.move_arm()
    node.move_gripper()
    node.get_logger().info("Done!")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
