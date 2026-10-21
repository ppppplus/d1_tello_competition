#!/usr/bin/env python3
"""Minimal RGB-only top-marker classification during the reference UAV patrol.
Uses public station geometry and UAV odometry, never referee tasks or run files.
"""
import json
import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile,DurabilityPolicy,qos_profile_sensor_data
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from std_msgs.msg import String

class Reporter(Node):
    def __init__(self):
        super().__init__('uav_color_report',automatically_declare_parameters_from_overrides=True)
        self.state=None;self.odom=None;self.last=None;self.count=0;self.sent=set()
        self.pub=self.create_publisher(String,'/uav/target_report',10)
        self.create_subscription(String,'/competition/state',self.feedback,QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(Odometry,'/uav/odom',self.position,10)
        self.create_subscription(Image,'/competition/uav/image',self.image,qos_profile_sensor_data)
    def feedback(self,m):self.state=json.loads(m.data)
    def position(self,m):self.odom=m
    def image(self,m):
        if self.state is None or self.odom is None or not self.state['uav_entered']:return
        if m.encoding not in ('rgb8','bgr8'):return
        if abs((m.header.stamp.sec+m.header.stamp.nanosec*1e-9)-(self.odom.header.stamp.sec+self.odom.header.stamp.nanosec*1e-9))>.25:return
        p=self.odom.pose.pose.position;q=self.odom.pose.pose.orientation
        if abs(q.x)>.025 or abs(q.y)>.025:return
        candidates=self.state['stations'];station=min(candidates,key=lambda s:math.hypot(p.x-s['marker_position'][0],p.y-s['marker_position'][1]))
        x,y,z=station['marker_position'];slot=station['slot']
        if slot in self.sent or math.hypot(p.x-x,p.y-y)>.10 or p.z<z+.6:return
        pixels=np.frombuffer(m.data,dtype=np.uint8).reshape(m.height,m.step)[:,:m.width*3].reshape(m.height,m.width,3)
        crop=pixels[m.height//2-10:m.height//2+10,m.width//2-10:m.width//2+10]
        rgb=np.median(crop,axis=(0,1)).astype(float)
        if m.encoding=='bgr8':rgb=rgb[::-1]
        dominant=int(np.argmax(rgb));others=np.delete(rgb,dominant)
        if dominant==2 and rgb[2]>1.6*max(rgb[0],rgb[1],1.):return
        color=('red','green','gray')[dominant] if rgb[dominant]>1.6*max(others.max(),1.) and rgb[dominant]>35 else 'gray'
        key=(slot,color);self.count=self.count+1 if key==self.last else 1;self.last=key
        if self.count<3:return
        self.pub.publish(String(data=json.dumps(dict(slot=slot,marker_color=color))))
        self.sent.add(slot);self.get_logger().info(f'Station {slot+1}: {color} (RGB image)')

def main():
    rclpy.init();node=Reporter()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:node.destroy_node();rclpy.shutdown()
if __name__=='__main__':main()
