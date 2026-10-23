#!/usr/bin/env python3
"""Transport skeleton only: receive observations and hold the robot still."""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String

class Contestant(Node):
    def __init__(self):
        super().__init__('contestant', automatically_declare_parameters_from_overrides=True)
        self.velocity = self.create_publisher(Twist, '/d1/cmd_vel', 1)
        self.odom = None
        self.state = None
        self.create_subscription(Odometry, '/d1/odom', self.on_odom, 10)
        self.create_subscription(String, '/competition/state', self.on_state, 10)
        self.create_timer(0.05, self.tick)

    def on_odom(self, msg):
        if self.odom is None:
            self.get_logger().info('Connected: received D1 odometry')
        self.odom = msg

    def on_state(self, msg):
        if self.state is None:
            self.get_logger().info('Connected: received competition state')
        self.state = msg

    def tick(self):
        # Implement perception, planning and control here. Default: zero velocity.
        self.velocity.publish(Twist())

def main():
    rclpy.init()
    node = Contestant()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.velocity.publish(Twist())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
