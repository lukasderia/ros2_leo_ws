#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import math

class ScanFilter(Node):
    def __init__(self):
        super().__init__('scan_filter')
        self.sub = self.create_subscription(LaserScan, '/scan', self.callback, 10)
        self.pub = self.create_publisher(LaserScan, '/scan_filter', 10)

    def callback(self, msg):
        # Replace NaN with inf
        if math.isnan(msg.ranges[0]):
            msg.ranges[0] = float('inf')
        if math.isnan(msg.intensities[0]):
            msg.intensities[0] = 0.0

        # Tweak the incremetns to make the number match slam
        N = len(msg.ranges)
        msg.angle_increment = (msg.angle_max - msg.angle_min) / (N-1)
        
        self.pub.publish(msg)

def main():
    rclpy.init()
    node = ScanFilter()
    rclpy.spin(node)

if __name__ == '__main__':
    main()