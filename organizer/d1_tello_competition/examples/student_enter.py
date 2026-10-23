#!/usr/bin/env python3
"""Known-map example: wait for the UAV report, then walk through the door."""
import json
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String


def clip(value, low, high):
    return min(high, max(low, value))


class StudentEnter(Node):
    def __init__(self):
        super().__init__('student_enter', automatically_declare_parameters_from_overrides=True)
        self.pose = None
        self.state = None
        self.phase = 'waiting'
        self.done = False
        self.failed = False
        self.pause_until = 0.
        self.publisher = self.create_publisher(Twist, '/d1/cmd_vel', 1)
        self.create_subscription(Odometry, '/d1/odom', self.odometry, 10)
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, '/competition/state', self.feedback, qos)
        self.create_timer(.05, self.update)

    def odometry(self, message):
        self.pose = message.pose.pose

    def feedback(self, message):
        self.state = json.loads(message.data)

    def update(self):
        command = Twist()
        if self.pose is None or self.state is None:
            return
        now = self.get_clock().now().nanoseconds * 1e-9
        if self.state['mode'] != 'student' or self.state['finished']:
            self.publisher.publish(command)
            self.failed = True
            self.done = True
            return
        if self.phase == 'waiting':
            if self.state['uav_entered'] and any(self.state['reported']):
                self.phase = 'turning'
                self.get_logger().info('First indoor target reported: entering the ground door.')
        q = self.pose.orientation
        yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        error = math.atan2(math.sin(math.pi/2-yaw), math.cos(math.pi/2-yaw))
        if self.phase == 'turning':
            if abs(error) < .055:
                self.phase = 'settling'
                self.pause_until = now+1.5
            else:
                command.angular.z = clip(1.5*error, -.5, .5)
                if abs(command.angular.z) < .13:
                    command.angular.z = math.copysign(.13, error)
        elif self.phase == 'settling' and now >= self.pause_until:
            self.phase = 'walking'
        elif self.phase == 'walking':
            entry = self.state['entry']
            remaining = entry['bounds_xy'][2]+.7-self.pose.position.y
            cross = self.pose.position.x-entry['door']['center_x']
            if remaining < .06 and self.state['d1_entered']:
                self.phase = 'stopping'
                self.pause_until = now+1.
            else:
                world_x = clip(-cross*.7, -.10, .10)
                world_y = clip(remaining*.8, .065, .22)
                command.linear.x = math.cos(yaw)*world_x+math.sin(yaw)*world_y
                command.linear.y = -math.sin(yaw)*world_x+math.cos(yaw)*world_y
                command.angular.z = clip(1.5*error, -.5, .5)
        elif self.phase == 'stopping' and now >= self.pause_until:
            self.get_logger().info('D1 is fully inside the building: stopped.')
            self.done = True
        self.publisher.publish(command)


def main():
    rclpy.init()
    node = StudentEnter()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.publisher.publish(Twist())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 1 if node.failed else 0


if __name__ == '__main__':
    raise SystemExit(main())
