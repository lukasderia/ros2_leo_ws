#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
import subprocess
import os
import signal
from datetime import datetime

class RealRecorder(Node):
    def __init__(self):
        super().__init__('recorder_node')
        
        self.declare_parameter('mode', 2)
        self.mode_int_ = self.get_parameter('mode').get_parameter_value().integer_value
        mode_map = {0: 'yamauchi', 1: 'gao', 2: 'rss'}
        self.mode_name_ = mode_map.get(self.mode_int_, 'unknown')

        self.recording_ = False
        self.bag_process_ = None
        self.bag_dir_ = None

        self.auto_mode_sub_ = self.create_subscription(
            Bool, '/auto_mode', self.auto_mode_callback, 10)

        self.get_logger().info(f'Real recorder ready. Mode: {self.mode_name_}')

    def auto_mode_callback(self, msg):
        if msg.data and not self.recording_:
            self.start_recording()
        elif not msg.data and self.recording_:
            self.stop_recording()

    def start_recording(self):
        timestamp = datetime.now().strftime('%d%b_%Hh%Mm%Ss').lower()
        session_dir = os.path.expanduser(f'~/ros2_leo_ws/bags/session_{self.mode_name_}')
        os.makedirs(session_dir, exist_ok=True)

        self.bag_dir_ = os.path.join(session_dir, f'run_{timestamp}')

        self.bag_process_ = subprocess.Popen([
            'ros2', 'bag', 'record', '-o', self.bag_dir_,
            '/tf', '/tf_static', '/odom', '/map',
            '/odometry_merged', '/rss', '/rss_gradient',
            '/cmd_vel', '/goal_pose', '/auto_mode'
        ])

        self.recording_ = True
        self.get_logger().info(f'Recording started: {self.bag_dir_}')

    def stop_recording(self):
        if self.bag_process_:
            self.bag_process_.send_signal(signal.SIGINT)
            self.bag_process_.wait()
            self.bag_process_ = None

        self.recording_ = False
        self.get_logger().info(f'Recording stopped: {self.bag_dir_}')

def main(args=None):
    rclpy.init(args=args)
    node = RealRecorder()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()