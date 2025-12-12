#!/usr/bin/env python3

import rclpy # main ros2 python lib
from rclpy.node import Node # base class for all node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from tf_transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import Quaternion



class odom2TF(Node):
    """A ROS2 node that takes Odometry from bridged ROS1 messages from Leo Rover and republishes them again with reference to Odom frame"""
    def __init__(self):
        super().__init__("odom2TF")
        self.tf_broadcaster = TransformBroadcaster(self)
        self.subscription = self.create_subscription(Odometry, '/odometry_merged', self.odom_callback, 10)

        self.get_logger().info('Odometry to TF started')

    def odom_callback(self, msg):
        self.get_logger().info('Recieved Odom msg')
        t = TransformStamped()

        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"
        t.header.stamp = self.get_clock().now().to_msg()
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z =  msg.pose.pose.position.z
        q = msg.pose.pose.orientation
        yaw = self.quat_to_yaw(q)          # extract yaw from quaternion
        t.transform.rotation = self.yaw_to_quat(yaw)

        # Publish transform
        self.tf_broadcaster.sendTransform(t)

    def quat_to_yaw(self, q):
        # q is geometry_msgs.msg.Quaternion
        roll, pitch, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        return yaw
    
    def yaw_to_quat(self, yaw):
        qx, qy, qz, qw = quaternion_from_euler(0.0, 0.0, yaw)
        return Quaternion(x=qx, y=qy, z=qz, w=qw)


def main(args=None):
    rclpy.init(args=args)
    node = odom2TF()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()