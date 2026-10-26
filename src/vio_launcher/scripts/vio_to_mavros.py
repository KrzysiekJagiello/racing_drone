#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped

class VioToMavros(Node):
    def __init__(self):
        super().__init__("vio_to_mavros")
        self.pub = self.create_publisher(PoseStamped, "/mavros/vision_pose/pose", 10)
        self.create_subscription(Odometry, "/ov_msckf/odomimu", self.cb, qos_profile_sensor_data)

    def cb(self, m):
        o = PoseStamped()
        o.header.stamp = m.header.stamp
        o.header.frame_id = "map"
        o.pose.position = m.pose.pose.position
        q = m.pose.pose.orientation
        o.pose.orientation.x = q.w
        o.pose.orientation.y = q.z
        o.pose.orientation.z = -q.y
        o.pose.orientation.w = -q.x
        self.pub.publish(o)

def main():
    rclpy.init()
    rclpy.spin(VioToMavros())

if __name__ == "__main__":
    main()