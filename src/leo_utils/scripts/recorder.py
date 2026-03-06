#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from nav_msgs.msg import OccupancyGrid
import subprocess
import signal
import os
import math
import time
from datetime import datetime
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
import tf2_ros
from geometry_msgs.msg import TransformStamped

class Recorder(Node):
    def __init__(self):
        super().__init__('recorder_node')

        # Parameters
        self.declare_parameter('robot_x', 0.0)
        self.declare_parameter('robot_y', 0.0)
        self.declare_parameter('router_x', 18.0)
        self.declare_parameter('router_y', 18.0)
        self.declare_parameter('mode', 'rss')
        self.declare_parameter('max_duration', 480.0)  # 5 minutes

        self.robot_x_param_ = self.get_parameter('robot_x').get_parameter_value().double_value
        self.robot_y_param_ = self.get_parameter('robot_y').get_parameter_value().double_value
        self.router_x_ = self.get_parameter('router_x').get_parameter_value().double_value
        self.router_y_ = self.get_parameter('router_y').get_parameter_value().double_value
        self.mode_ = self.get_parameter('mode').get_parameter_value().string_value
        self.max_duration_ = self.get_parameter('max_duration').get_parameter_value().double_value

        # QoS for auto_mode
        qos = QoSProfile(
            depth=10,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE
        )

        # Publishers/Subscribers
        self.auto_mode_pub_ = self.create_publisher(Bool, '/auto_mode', qos)
        self.map_sub_ = self.create_subscription(OccupancyGrid, '/map_filtered', self.map_callback, 10)

        # TF
        self.tf_buffer_ = tf2_ros.Buffer()
        self.tf_listener_ = tf2_ros.TransformListener(self.tf_buffer_, self)

        # State
        self.bag_process = None
        self.latest_map_ = None
        self.start_time_ = None
        self.done_ = False

        # Wait 30s for system to initialize then start
        self.init_timer_ = self.create_timer(10.0, self.start)
        self.get_logger().info("Recorder waiting for system to initialize...")

    def start(self):
        self.init_timer_.cancel()
        self.get_logger().info("Starting exploration...")

        # Enable auto mode
        msg = Bool()
        msg.data = True
        self.auto_mode_pub_.publish(msg)

        # Start recording
        self.start_recording()

        # Start check timer
        self.start_time_ = time.time()
        self.check_timer_ = self.create_timer(1.0, self.check_termination)

    def map_callback(self, msg):
        self.latest_map_ = msg

    def check_termination(self):
        if self.done_:
            return

        # Check timeout
        elapsed = time.time() - self.start_time_
        if elapsed > self.max_duration_:
            self.get_logger().info("Timeout reached - stopping")
            self.shutdown()
            return

        # Get robot position from TF
        try:
            transform = self.tf_buffer_.lookup_transform('map', 'base_link', rclpy.time.Time())
            robot_x = transform.transform.translation.x
            robot_y = transform.transform.translation.y
        except Exception as e:
            return

        # Check distance to router
        dx = robot_x - self.router_x_
        dy = robot_y - self.router_y_
        distance = math.sqrt(dx*dx + dy*dy)

        if distance < 3.0 and self.is_router_cell_free():
            self.get_logger().info(f"Router found! Distance: {distance:.2f}m")
            self.shutdown()

    def is_router_cell_free(self):
        if self.latest_map_ is None:
            return False

        cell_x = int((self.router_x_ - self.latest_map_.info.origin.position.x) / self.latest_map_.info.resolution)
        cell_y = int((self.router_y_ - self.latest_map_.info.origin.position.y) / self.latest_map_.info.resolution)

        if cell_x < 0 or cell_x >= self.latest_map_.info.width or \
           cell_y < 0 or cell_y >= self.latest_map_.info.height:
            return False

        index = cell_y * self.latest_map_.info.width + cell_x
        cell_value = self.latest_map_.data[index]
        return 0 <= cell_value < 40

    def start_recording(self):
            date_str = datetime.now().strftime("%d%b_%Hh%M")
            bag_name = f"Recording_{self.mode_}_rx{self.router_x_}_ry{self.router_y_}_bx{self.robot_x_param_}_by{self.robot_y_param_}_{date_str}"
            
            mode_dir = os.path.expanduser(f"~/ros2_leo_ws/bags/session_{self.mode_}")
            os.makedirs(mode_dir, exist_ok=True)
            bag_path = os.path.join(mode_dir, bag_name)

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

    def shutdown(self):
        self.done_ = True
        self.check_timer_.cancel()

    def stop_recording(self):
        if self.bag_process:
            self.bag_process.send_signal(signal.SIGINT)
            self.bag_process.wait()
            self.bag_process = None
            self.get_logger().info("Recording stopped")

def main(args=None):
    rclpy.init(args=args)
    node = Recorder()
    
    while rclpy.ok() and not node.done_:
        rclpy.spin_once(node, timeout_sec=0.1)
    
    # Shutdown cleanly outside callback
    msg = Bool()
    msg.data = False
    node.auto_mode_pub_.publish(msg)
    node.stop_recording()
    node.get_logger().info("Done - shutting down")
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()