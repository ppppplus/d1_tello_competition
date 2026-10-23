#!/usr/bin/env python3
"""Minimal ROS 2 student example: wait, walk for 3 s, turn for 2 s, stop."""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class StudentWalk(Node):
    def __init__(self):
        super().__init__('student_walk', automatically_declare_parameters_from_overrides=True)
        self.publisher = self.create_publisher(Twist, '/d1/cmd_vel', 1)
        self.start = None
        self.done = False
        self.timer = self.create_timer(.05, self.update)

    def update(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        if now <= 0:
            return
        if self.start is None:
            self.start = now
        elapsed = now - self.start
        msg = Twist()
        if 2 <= elapsed < 5:
            msg.linear.x = .2
        elif 6 <= elapsed < 8:
            msg.angular.z = .4
        self.publisher.publish(msg)
        if elapsed >= 9:
            self.get_logger().info('Example complete: stopped.')
            self.done = True
            self.timer.cancel()


def main():
    rclpy.init()
    node = StudentWalk()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=.1)
    except KeyboardInterrupt:
        node.publisher.publish(Twist())
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
