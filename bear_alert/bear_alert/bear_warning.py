#!/usr/bin/env python3

"""
Bear Warning Node

Subscribes to '/bear_alert' (std_msgs/Bool). While the alert is on, CRANE+ V2
opens its hand and waves its arm left and right. When the alert turns off,
the arm returns to the rest pose and the hand closes. If no alert message arrives for
`timeout` seconds (e.g. the detector stopped), the alert is treated as off.

Arm commands go to 'crane_plus_arm_controller/joint_trajectory' (same as the
physicalai_pubsub action_publisher of lecture 4), and gripper commands go to
'crane_plus_gripper_controller/joint_trajectory'.

Example:
    ros2 run bear_alert bear_warning
"""

import rclpy
from builtin_interfaces.msg import Duration
from rclpy.node import Node
from std_msgs.msg import Bool
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

ARM_JOINTS = [
    'crane_plus_joint1',
    'crane_plus_joint2',
    'crane_plus_joint3',
    'crane_plus_joint4',
]
GRIPPER_JOINT = 'crane_plus_joint_hand'


class BearWarning(Node):
    """ROS2 node that makes CRANE+ V2 perform a warning pose while a bear is detected."""

    def __init__(self):
        super().__init__('bear_warning')

        # Joint positions [rad] for joint1..joint4
        self.declare_parameter('rest_pose', [0.0, 0.0, 0.0, 0.0])
        self.declare_parameter('wave_left', [0.8, 0.0, 0.0, 0.6])
        self.declare_parameter('wave_right', [-0.8, 0.0, 0.0, -0.6])
        # crane_plus_joint_hand: 0.0 = open, larger = closed (stops at about 0.67)
        self.declare_parameter('gripper_open', 0.0)
        self.declare_parameter('gripper_close', 0.6)
        self.declare_parameter('step', 1.5)  # time per wave motion [s]
        self.declare_parameter('timeout', 3.0)  # treat alert as OFF if no message for this long [s]

        self.rest_pose = list(self.get_parameter('rest_pose').value)
        self.wave_poses = [
            list(self.get_parameter('wave_left').value),
            list(self.get_parameter('wave_right').value),
        ]
        self.gripper_open = self.get_parameter('gripper_open').value
        self.gripper_close = self.get_parameter('gripper_close').value
        self.step = self.get_parameter('step').value
        self.timeout = self.get_parameter('timeout').value

        self.arm_pub = self.create_publisher(
            JointTrajectory, 'crane_plus_arm_controller/joint_trajectory', 10)
        self.gripper_pub = self.create_publisher(
            JointTrajectory, 'crane_plus_gripper_controller/joint_trajectory', 10)
        self.subscription = self.create_subscription(
            Bool, 'bear_alert', self.on_alert, 10)

        self.alert = False
        self.wave_index = 0
        self.last_msg_time = self.get_clock().now()
        self.timer = self.create_timer(self.step, self.on_timer)

        self.get_logger().info('Bear Warning started')
        self.move_arm(self.rest_pose, 2.0)
        self.move_gripper(self.gripper_close, 1.0)

    def on_alert(self, msg):
        self.last_msg_time = self.get_clock().now()
        self.set_alert(msg.data)

    def set_alert(self, value):
        if value and not self.alert:
            self.get_logger().warn('Alert ON -> warning pose')
            # Open the hand and keep it open while the alert is on
            self.move_gripper(self.gripper_open, 1.0)
        elif not value and self.alert:
            self.get_logger().info('Alert OFF -> back to rest pose')
            self.move_arm(self.rest_pose, 2.0)
            self.move_gripper(self.gripper_close, 1.0)
        self.alert = value

    def on_timer(self):
        # Fail-safe: if the detector stops sending, do not keep waving forever
        elapsed = (self.get_clock().now() - self.last_msg_time).nanoseconds / 1e9
        if self.alert and elapsed > self.timeout:
            self.get_logger().warn('No alert message received -> treat as OFF')
            self.set_alert(False)
        if not self.alert:
            return
        self.move_arm(self.wave_poses[self.wave_index], self.step)
        self.wave_index = 1 - self.wave_index

    def move_arm(self, positions, seconds):
        self.arm_pub.publish(self.make_trajectory(ARM_JOINTS, positions, seconds))

    def move_gripper(self, position, seconds):
        self.gripper_pub.publish(self.make_trajectory([GRIPPER_JOINT], [position], seconds))

    @staticmethod
    def make_trajectory(joint_names, positions, seconds):
        msg = JointTrajectory()
        msg.joint_names = joint_names
        point = JointTrajectoryPoint()
        point.positions = [float(p) for p in positions]
        point.time_from_start = Duration(sec=int(seconds), nanosec=int((seconds % 1) * 1e9))
        msg.points.append(point)
        return msg


def main():
    rclpy.init()
    node = BearWarning()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
