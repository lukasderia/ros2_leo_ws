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
import json
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
        self.declare_parameter('max_duration', 600.0)

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
        self.termination_reason_ = 'unknown'
        self.bag_path_ = None
        self.bag_name_ = None

        # Wait for system to initialize then start
        self.init_timer_ = self.create_timer(10.0, self.start)
        self.get_logger().info("Recorder waiting for system to initialize...")

    def start(self):
        self.init_timer_.cancel()
        self.get_logger().info("Starting exploration...")

        msg = Bool()
        msg.data = True
        self.auto_mode_pub_.publish(msg)

        self.start_recording()

        self.start_time_ = time.time()
        self.check_timer_ = self.create_timer(1.0, self.check_termination)

    def map_callback(self, msg):
        self.latest_map_ = msg

    def check_termination(self):
        if self.done_:
            return

        elapsed = time.time() - self.start_time_
        if elapsed > self.max_duration_:
            self.get_logger().info("Timeout reached - stopping")
            self.termination_reason_ = 'timeout'
            self.shutdown()
            return

        try:
            transform = self.tf_buffer_.lookup_transform('map', 'base_link', rclpy.time.Time())
            robot_x = transform.transform.translation.x
            robot_y = transform.transform.translation.y

            # Extract roll and pitch from quaternion
            q = transform.transform.rotation
            # Roll
            sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z)
            cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
            roll = math.atan2(sinr_cosp, cosr_cosp)

            # Pitch
            sinp = 2.0 * (q.w * q.y - q.z * q.x)
            pitch = math.asin(max(-1.0, min(1.0, sinp)))

            roll_deg = math.degrees(abs(roll))
            pitch_deg = math.degrees(abs(pitch))

            if roll_deg > 25.0 or pitch_deg > 25.0:
                self.get_logger().info(f"Flip detected! Roll: {roll_deg:.1f} Pitch: {pitch_deg:.1f}")
                self.termination_reason_ = 'flip'
                self.shutdown()
                return

        except Exception as e:
            return

        dx = robot_x - self.router_x_
        dy = robot_y - self.router_y_
        distance = math.sqrt(dx*dx + dy*dy)

        if distance < 3.0 and self.is_router_cell_free():
            self.get_logger().info(f"Router found! Distance: {distance:.2f}m")
            self.termination_reason_ = 'router_found'
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
        self.bag_name_ = f"Recording_{self.mode_}_rx{self.router_x_}_ry{self.router_y_}_bx{self.robot_x_param_}_by{self.robot_y_param_}_{date_str}"

        mode_dir = os.path.expanduser(f"~/ros2_leo_ws/bags/session_{self.mode_}")
        os.makedirs(mode_dir, exist_ok=True)
        self.bag_path_ = os.path.join(mode_dir, self.bag_name_)

        topics = [
            "/weak_signal_state", "/rss", "/rss_gradient",
            "/firmware/battery_averaged", "/auto_mode", "/cmd_vel",
            "/frontier_centroid_markers", "/frontier_centroids", "/frontier_markers",
            "/global_costmap/costmap", "/goal_pose", "/local_costmap/costmap",
            "/map", "/map_filtered", "/odometry_merged", "/plan", "/tf", "/tf_static",
            "/robot_description", "/odom", "/joint_states",
            "/received_global_plan", "/gradient_arrow"
        ]

        cmd = ["ros2", "bag", "record", "-o", self.bag_path_] + topics
        self.bag_process = subprocess.Popen(cmd)
        self.get_logger().info(f"Started recording: {self.bag_name_}")

    def write_result(self):
        if self.bag_path_ is None:
            return

        duration = time.time() - self.start_time_ if self.start_time_ else 0.0

        result = {
            "bag_name": self.bag_name_,
            "termination_reason": self.termination_reason_,
            "duration_seconds": round(duration, 1),
            "mode": self.mode_,
            "robot_start": [self.robot_x_param_, self.robot_y_param_],
            "router_position": [self.router_x_, self.router_y_],
            "timestamp": datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        }

        result_path = os.path.join(self.bag_path_, "result.json")
        with open(result_path, 'w') as f:
            json.dump(result, f, indent=2)

        self.get_logger().info(f"Result written: {self.termination_reason_} in {duration:.1f}s")

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

    msg = Bool()
    msg.data = False
    node.auto_mode_pub_.publish(msg)
    node.stop_recording()
    node.write_result()
    node.get_logger().info("Done - shutting down")
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()