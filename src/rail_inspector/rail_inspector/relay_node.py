#! /usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from geometry_msgs.msg import TransformStamped
import tf2_ros


class RelayNode(Node):
    def __init__(self):
        super().__init__('odom_relay')
        self.sub_odom = self.create_subscription(Odometry, '/leg_odom2', self.cb_odom, 10)
        self.pub_odom = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

    def cb_odom(self, msg):
        msg.header.frame_id = "odom"
        msg.child_frame_id = 'base_link'
        msg.header.stamp = self.get_clock().now().to_msg()
        self.pub_odom.publish(msg)

        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = "odom"
        t.child_frame_id = 'base_link'
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        t.transform.rotation = msg.pose.pose.orientation
        self.tf_broadcaster.sendTransform(t)

def main():
    rclpy.init()
    node = RelayNode()
    rclpy.spin(node)
    rclpy.shutdown()
    
main()