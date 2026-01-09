#!/usr/bin/env python3

import rclpy # main ros2 python lib
from rclpy.node import Node # base class for all node
from nav_msgs.msg import OccupancyGrid
import cv2
import numpy as np

class MapFilter(Node):
    def __init__(self):
        self.recievedFirstMap = False
        super().__init__('map_filter')
        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)
        self.filtered_map_pub = self.create_publisher(OccupancyGrid, '/map_filtered', 10)
        
        self.latest_map = None
        
        # Timer
        self.timer = self.create_timer(1.0, self.process_and_publish)

    def map_callback(self, msg):
        # Just store the latest map
        self.latest_map = msg
        if not self.recievedFirstMap:
            self.get_logger().info('First map received.')
            self.recievedFirstMap = True

    def process_and_publish(self):
        if self.latest_map is None:
            return
        
        msg = self.latest_map
        width = msg.info.width
        height = msg.info.height
        data = np.array(msg.data, dtype=np.int8).reshape((height, width))
        
        # Step 1: Process obstacles FIRST
        map_with_thick_obstacles = self.smoothObstacles(data)
        
        # Step 2: Overlay thickened obstacles
        combined = np.copy(data)
        obstacle_mask = (map_with_thick_obstacles == 100)
        combined[obstacle_mask] = 100
        
        # Step 3: Smooth free space
        map_smooth = self.smoothFreeSpace(combined)
        
        # Publish
        filtered_map_msg = OccupancyGrid()
        filtered_map_msg.header = msg.header
        filtered_map_msg.info = msg.info
        filtered_map_msg.data = map_smooth.flatten().tolist()
        self.filtered_map_pub.publish(filtered_map_msg)

    def smoothFreeSpace(self, data):
        map_copy = np.copy(data)
        free_binary = (map_copy == 0).astype(np.uint8) * 255 # type: ignore
        obstacle_mask = (map_copy == 100)  # Don't touch obstacles
        
        # Your existing closing logic...
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        closed = cv2.morphologyEx(free_binary, cv2.MORPH_CLOSE, kernel, iterations=2)
        
        # Make sure obstacles stay obstacles
        closed[obstacle_mask] = 0
        
        result = np.where(obstacle_mask, 100,
                        np.where(closed == 255, 0, -1))
        return result.astype(np.int8) # type: ignore

    def smoothObstacles(self, data):
        map_onlyObstacles = np.copy(data)
        # Extract obstacles: values > 0
        obstacles_binary = (map_onlyObstacles > 0).astype(np.uint8) * 255 # type: ignore
        
        # Morphological closing to fill gaps and make consistent
        kernel = np.ones((3, 3), np.uint8) # type: ignore
        closed = cv2.morphologyEx(obstacles_binary, cv2.MORPH_CLOSE, kernel, iterations=2)
        
        # Convert back to occupancy format (100 for obstacles)
        result = np.where(closed == 255, 100, 0)
        
        return result.astype(np.int8) # type: ignore

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