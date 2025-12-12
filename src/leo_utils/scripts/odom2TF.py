#!/usr/bin/env python3

import rclpy # main ros2 python lib
from rclpy.node import Node # base class for all node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


class odom2TF(Node):
    """A ROS2 node that takes Odometry from bridged ROS1 messages from Leo Rover and republishes them again with reference to Odom frame"""
    def __init__(self):
        super().__init__("odom2TF")
        self.tf_broadcaster = TransformBroadcaster(self)
        self.subscription = self.create_subscription(Odometry, '/odometry_merged', self.odom_callback, 10)

        self.get_logger().info('Odometry to TF started')

    def odom_callback(self, msg):
        t = TransformStamped()
        # Copy headers and posse to re-broadcast them
        t.header = msg.header
        t.header.stamp = msg.header.stamp #self.get_clock().now().to_msg()
        t.child_frame_id = msg.child_frame_id
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        t.transform.rotation = msg.pose.pose.orientation

        # Publish transform
        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = odom2TF()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()