#!/usr/bin/env python3
from rosbags.rosbag2 import Reader
from rosbags.serde import deserialize_cdr
import cv2
import numpy as np


class MapFilterStandalone:
    def __init__(self, show=True, win_size=(1200, 800)):
        self.show = show
        self.win_size = win_size

    # -----------------------
    # Bag IO (like "subscription")
    # -----------------------
    def read_last_map_from_bag(self, bag_path, topic="/map"):
        with Reader(bag_path) as reader:
            last_msg = None
            last_conn = None
            for connection, timestamp, rawdata in reader.messages():
                if connection.topic == topic:
                    last_msg = deserialize_cdr(rawdata, connection.msgtype)
                    last_conn = connection

        if last_msg is None:
            raise RuntimeError(f"No map message found on topic '{topic}' in bag: {bag_path}")

        width = last_msg.info.width
        height = last_msg.info.height
        data = np.array(last_msg.data, dtype=np.int8).reshape((height, width))

        return last_msg, data  # msg kept if you want metadata

    # -----------------------
    # Same processing pipeline as your ROS2 node
    # -----------------------
    def process(self, occupancy_data: np.ndarray) -> np.ndarray:
        # Remove obstacles and apply smoothing
        map_removedObstacles_smooth = self.smoothFreeSpace(occupancy_data)

        # Apply filtering on the obstacles
        map_smoothObstacles = self.smoothObstacles(occupancy_data)

        # Combine the two maps
        map_smooth = self.combineMaps(map_removedObstacles_smooth, map_smoothObstacles)

        return map_smooth.astype(np.int8)

    # def smoothFreeSpace(self, data: np.ndarray) -> np.ndarray:
    #     # 0=free, -1=unknown, >0=occupied
    #     map_removedObstacles = np.copy(data)
    #     map_removedObstacles[map_removedObstacles > 0] = 0  # replace obstacles with free

    #     free_binary = (map_removedObstacles == 0).astype(np.uint8) * 255

    #     kernel = np.ones((2, 2), np.uint8)
    #     opened = cv2.morphologyEx(free_binary, cv2.MORPH_OPEN, kernel, iterations=3)
    
    def smoothFreeSpace(self, data):
        # Create binary map: 255=free, 0=everything else
        map_copy = np.copy(data)
        free_binary = (map_copy == 0).astype(np.uint8) * 255
        
        # CLOSING instead of opening - fills gaps between sparse beams
        # Larger kernel to bridge distance between sparse laser beams
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        closed = cv2.morphologyEx(free_binary, cv2.MORPH_CLOSE, kernel, iterations=5)
        
        # Convert back: closed regions become free, rest stays unknown
        result = np.where(closed == 255, 0, -1)
        
        return result.astype(np.int8)

    def smoothObstacles(self, data: np.ndarray) -> np.ndarray:
        obstacles_binary = (data > 0).astype(np.uint8) * 255

        kernel = np.ones((3, 3), np.uint8)
        closed = cv2.morphologyEx(obstacles_binary, cv2.MORPH_CLOSE, kernel, iterations=1)

        result = np.where(closed == 255, 100, 0)  # 100=occupied, 0=free
        return result.astype(np.int8)

    def combineMaps(self, map_removedObstacles_smooth: np.ndarray, map_smoothObstacles: np.ndarray) -> np.ndarray:
        result = np.copy(map_removedObstacles_smooth)
        obstacle_mask = (map_smoothObstacles == 100)
        result[obstacle_mask] = 100
        return result

    # -----------------------
    # Visualization helpers
    # -----------------------
    def occupancy_to_image(self, occ: np.ndarray) -> np.ndarray:
        """
        Display mapping similar to your old script:
          occupied -> 0 (black)
          unknown  -> 205 (gray)
          free     -> 254 (light)
        """
        img = np.zeros_like(occ, dtype=np.uint8)
        img[occ == 0] = 254
        img[occ == -1] = 205
        img[occ > 0] = 0
        return img

    def show_image(self, title: str, img: np.ndarray, wait=True):
        if not self.show:
            return
        cv2.namedWindow(title, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(title, self.win_size[0], self.win_size[1])
        cv2.imshow(title, img)
        if wait:
            cv2.waitKey(0)

    # -----------------------
    # End-to-end run
    # -----------------------
    def run(self, bag_path: str):
        msg, occ = self.read_last_map_from_bag(bag_path, topic="/map")

        print(f"Map size: {msg.info.width}x{msg.info.height}")
        print(f"Resolution: {msg.info.resolution} m/cell")

        # Original
        self.show_image("Original (from bag)", self.occupancy_to_image(occ))

        # Node-like intermediate steps
        free_smoothed = self.smoothFreeSpace(occ)
        self.show_image("Free/Unknown smoothed", self.occupancy_to_image(free_smoothed))

        obstacles_smoothed = self.smoothObstacles(occ)
        self.show_image("Obstacles smoothed (binary-ish)", self.occupancy_to_image(obstacles_smoothed))

        combined = self.combineMaps(free_smoothed, obstacles_smoothed)
        self.show_image("Final combined", self.occupancy_to_image(combined))

        cv2.destroyAllWindows()
        return combined


if __name__ == "__main__":
    bag_path = "/home/lukas/ros2_ws/test_map_bag2"
    filt = MapFilterStandalone(show=True)
    filtered_occ = filt.run(bag_path)
