#!/usr/bin/env python3

import rclpy # main ros2 python lib
from rclpy.node import Node # base class for all node
from nav_msgs.msg import OccupancyGrid
import cv2
import numpy as np

class MapFilter (Node):
    def __init__(self):
        super().__init__('map_filter')
        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)

        self.filtered_map_pub = self.create_publisher(OccupancyGrid, '/map_filtered', 10)

    def map_callback(self, msg):
        # Here recieve the map and convert it to be fitlered
        width = msg.info.width
        height = msg.info.height

        # Convert the map to numpy array for opencv
        map_array = np.array(msg.data, dtype=np.int8).reshape((height, width)) # type: ignore
        filtered_array = self.apply_filtering(map_array)

        filtered_msg = OccupancyGrid()
        filtered_msg.header = msg.header
        filtered_msg.info = msg.info
        filtered_msg.data = filtered_array.flatten().tolist()

        self.filtered_map_pub.publish(filtered_msg)



    def apply_filtering(self, map_array):
        # Prepare map for OpenCV (needs uint8)
        # Convert: -1 (unknown) → 205, 0-100 (occupancy) → 0-100, free space → 255
        filtered = np.copy(map_array)
        
        # Remap values for processing
        filtered[map_array == -1] = 205  # Unknown
        filtered[map_array == 0] = 255   # Free space
        # Occupied cells (1-100) stay as-is (closer to 0 = black obstacles)
        
        # Convert to uint8 for OpenCV
        filtered = filtered.astype(np.uint8)
        
        # Bilateral filter parameters (tune these)
        d = 5           # Diameter of pixel neighborhood
        sigma_color = 50   # Filter sigma in the color space (σr)
        sigma_space = 50   # Filter sigma in the coordinate space (σs)
        
        # Apply bilateral filter
        filtered = cv2.bilateralFilter(filtered, d, sigma_color, sigma_space)
        
        # Dilation to expand free space and remove small unknown regions
        kernel_size = 3  # Size of dilation kernel
        kernel = np.ones((kernel_size, kernel_size), np.uint8)
        filtered = cv2.dilate(filtered, kernel, iterations=1)
        
        # Convert back to occupancy grid format
        result = np.copy(filtered).astype(np.int8)
        result[filtered == 205] = -1   # Unknown
        result[filtered == 255] = 0    # Free
        # Occupied stays as-is
        
        return result

def main(args=None):
    rclpy.init(args=args)
    node = MapFilter()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__=='__main__':
    main()