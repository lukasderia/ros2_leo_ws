#!/usr/bin/env python3

import rclpy # main ros2 python lib
from rclpy.node import Node # base class for all node
from nav_msgs.msg import OccupancyGrid
import cv2
import numpy as np

class MapFilter (Node):


    def __init__(self):
        self.recievedFirstMap = False
        super().__init__('map_filter')
        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)
        self.filtered_map_pub = self.create_publisher(OccupancyGrid, '/map_filtered', 10)

    def map_callback(self, msg):
        if not self.recievedFirstMap:
            self.get_logger().info('First map recieved. Processing...')
            self.recievedFirstMap = True

        # Extract data
        width = msg.info.width
        height = msg.info.height
        data = np.array(msg.data, dtype=np.int8).reshape((height, width))
        
        # Remove obstacles and apply smoothing
        map_removedObstacles_smooth = self.smoothFreeSpace(data)

        # Apply filtering on the obstacles
        map_smoothObstacles = self.smoothObstacles(data)

        # Combine the two maps
        map_smooth = self.combineMaps(map_removedObstacles_smooth, map_smoothObstacles)

        # Create new msg to be published wth the same headers as original
        filtered_map_msg = OccupancyGrid()
        filtered_map_msg.header = msg.header
        filtered_map_msg.info = msg.info
        filtered_map_msg.data = map_smooth.flatten().tolist()  # Must be flat list

        self.filtered_map_pub.publish(filtered_map_msg)

    def smoothFreeSpace(self, data):
        # occupancy_data_no_obstacles has: 0=free, -1=unknown
        # Convert to binary: 255=free, 0=unknown

        map_removedObstacles = np.copy(data)
        map_removedObstacles[map_removedObstacles > 0] = 0  # Replace all obstacles with free 

        free_binary = (map_removedObstacles == 0).astype(np.uint8) * 255
        
        # Opening: remove stray pixels, smooth boundaries
        kernel = np.ones((3, 3), np.uint8)
        opened = cv2.morphologyEx(free_binary, cv2.MORPH_OPEN, kernel, iterations=1)
        
        # Convert back to occupancy format
        result = np.where(opened == 255, 0, -1)  # 0=free, -1=unknown
        
        return result.astype(np.int8)
    
    def smoothObstacles(self, data):

        map_onlyObstacles = np.copy(data)
        # Extract obstacles: values > 0
        obstacles_binary = (map_onlyObstacles > 0).astype(np.uint8) * 255
        
        # Morphological closing to fill gaps and make consistent
        kernel = np.ones((9, 9), np.uint8)
        closed = cv2.morphologyEx(obstacles_binary, cv2.MORPH_CLOSE, kernel, iterations=1)
        
        # Convert back to occupancy format (100 for obstacles)
        result = np.where(closed == 255, 100, 0)
        
        return result.astype(np.int8)

    def combineMaps(self, map_removedObstacles_smooth, map_smoothObstacles):
        # Start with smoothed free/unknown (0=free, -1=unknown)
        result = np.copy(map_removedObstacles_smooth)
        
        # Put obstacles on top (obstacles are 100 in map_smoothObstacles)
        obstacle_mask = (map_smoothObstacles == 100)
        result[obstacle_mask] = 100  # Obstacles
        
        return result
    
def main(args=None):
    rclpy.init(args=args)
    node = MapFilter()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__=='__main__':
    main()