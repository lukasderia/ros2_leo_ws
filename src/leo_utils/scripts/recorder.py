#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
import subprocess
import signal
import os
from datetime import datetime

class Recorder(Node):
    def __init__(self):
        super().__init__('recorder_node')
        self.mode_sub = self.create_subscription(Bool, "/auto_mode", self.bag_callback, 10)
        self.mode = False
        self.bag_process = None

    def bag_callback(self, msg):
        if msg.data and not self.mode:
            self.mode = True
            self.start_recording()
        elif not msg.data and self.mode:
            self.mode = False
            self.stop_recording()

    def start_recording(self):
        date_str = datetime.now().strftime("%d%b_%Hh%M")
        bag_name = f"Recording_{date_str}"
        bag_path = os.path.expanduser(f"~/ros2_leo_ws/bags/{bag_name}")

        topics = [
            "/weak_signal_state", "/rss", "/rss_gradient",
            "/firmware/battery_averaged", "/auto_mode", "/cmd_vel",
            "/frontier_centroid_markers", "/frontier_centroids", "/frontier_markers",
            "/global_costmap/costmap", "/goal_pose", "/local_costmap/costmap",
            "/map", "/map_filtered", "/odometry_merged", "/plan", "/tf", "/tf_static"
        ]

        cmd = ["ros2", "bag", "record", "-o", bag_path] + topics
        self.bag_process = subprocess.Popen(cmd)
        self.get_logger().info(f"Started recording: {bag_name}")

    def stop_recording(self):
        if self.bag_process:
            self.bag_process.send_signal(signal.SIGINT)
            self.bag_process.wait()
            self.bag_process = None
            self.get_logger().info("Recording stopped and saved")

def main(args=None):
    rclpy.init(args=args)
    node = Recorder()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()