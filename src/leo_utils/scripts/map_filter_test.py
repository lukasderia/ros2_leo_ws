#!/usr/bin/env python3
from rosbags.rosbag2 import Reader
from rosbags.serde import deserialize_cdr
import cv2
import numpy as np

def bag_to_image(bag_path):
    # Open bag with rosbags library
    with Reader(bag_path) as reader:
        # Find map messages
        map_msg = None
        for connection, timestamp, rawdata in reader.messages():
            if connection.topic == '/map':
                msg = deserialize_cdr(rawdata, connection.msgtype)
                map_msg = msg
        
        if not map_msg:
            print("No map message found in bag!")
            return None
        
        # Extract data
        width = map_msg.info.width
        height = map_msg.info.height
        data = np.array(map_msg.data, dtype=np.int8).reshape((height, width))
        
        print(f"Map size: {width}x{height}")
        print(f"Resolution: {map_msg.info.resolution} m/cell")
        
        # Convert to image
        image = np.zeros_like(data, dtype=np.uint8)
        image[data == 0] = 254      # Free
        image[data == -1] = 205     # Unknown  
        image[data > 0] = 0         # Occupied
        
        return image, data
    
def removeBlack(data):
   
    result = np.copy(data)
    result[result > 0] = 0  # Replace all obstacles with free space
    
    # Convert to image
    image = np.zeros_like(result, dtype=np.uint8)
    image[result == 0] = 254      # Free
    image[result == -1] = 205     # Unknown  
    
    return image

def excludeBlack(data):
    result = np.copy(data)
    result[result <= 0] = 0

    #convert back to image
    image = np.zeros_like(result, dtype=np.uint8)
    image[result == 0] = 254      # Free
    image[result == -1] = 205     # Unknown  
    image[result > 0] = 0         # Occupied

    return image

def make_obstacles_consistent_on_image(image):
    # Work directly on the image
    # Assume: 0=black (obstacles), 254=light (free/background)
    
    # Invert so obstacles are white (255) for morphology
    inverted = 255 - image
    
    # Morphological operations
    kernel = np.ones((9, 9), np.uint8)
    closed = cv2.morphologyEx(inverted, cv2.MORPH_CLOSE, kernel, iterations=1)
    
    # Invert back (obstacles=black agaisn)
    result = 255 - closed
    
    return result

def smooth_free_space_on_image(image_no_obstacles):
    # image_no_obstacles has: 254=free, 205=unknown
    # Convert to binary: 255=free, 0=unknown
    free_binary = (image_no_obstacles == 254).astype(np.uint8) * 255
    
    # Closing: fill gaps in free space
    kernel = np.ones((3, 3), np.uint8)
    #closed = cv2.morphologyEx(free_binary, cv2.MORPH_CLOSE, kernel, iterations=2)
    opened = cv2.morphologyEx(free_binary, cv2.MORPH_OPEN, kernel, iterations=1)
    
    # Convert back to image format
    result = np.where(opened == 255, 254, 205)  # 254=free, 205=unknown
    
    return result.astype(np.uint8)


def combine_layers(image_smoothed, image_consistent_obstacles):
    # Start with smoothed free/unknown
    result = np.copy(image_smoothed)
    
    # Put black obstacles on top (obstacles are 0=black in image_consistent_obstacles)
    obstacle_mask = (image_consistent_obstacles == 0)
    result[obstacle_mask] = 0  # Black = obstacles
    
    return result

if __name__ == '__main__':
    bag_path = '/home/lukas/ros2_ws/test_map_bag'
    
    result = bag_to_image(bag_path)
    if result:
        image, occupancy_data = result

        # Create resizable window
        cv2.namedWindow('Map from bag', cv2.WINDOW_NORMAL)
        cv2.resizeWindow('Map from bag', 1200, 800)
        cv2.imshow('Map from bag', image)
        print("Press any key to close...")
        cv2.waitKey(0)

        image_noBlack = removeBlack(occupancy_data)
        cv2.namedWindow('Map Without obstacles', cv2.WINDOW_NORMAL)
        cv2.resizeWindow('Map Without obstacles', 1200, 800)
        cv2.imshow('Map Without obstacles', image_noBlack)
        print("Press any key to close...")
        cv2.waitKey(0)

        # Smoothed
        image_smoothed = smooth_free_space_on_image(image_noBlack)
        cv2.namedWindow('Smoothed Free Space', cv2.WINDOW_NORMAL)
        cv2.resizeWindow('Smoothed Free Space', 1200, 800)
        cv2.imshow('Smoothed Free Space', image_smoothed)
        cv2.waitKey(0)

        image_onlyBlack = excludeBlack(occupancy_data)
        cv2.namedWindow('Map With only obstacles', cv2.WINDOW_NORMAL)
        cv2.resizeWindow('Map With only obstacles', 1200, 800)
        cv2.imshow('Map With only obstacles', image_onlyBlack)
        print("Press any key to close...")
        cv2.waitKey(0)
        
        # # Make obstacles consistent
        image_consistent_obstacles = make_obstacles_consistent_on_image(image_onlyBlack)
        cv2.namedWindow('Consistent obstacles', cv2.WINDOW_NORMAL)
        cv2.resizeWindow('Consistent obstacles', 1200, 800)
        cv2.imshow('Consistent obstacles', image_consistent_obstacles)
        cv2.waitKey(0)

        # Combined final result
        final_image = combine_layers(image_smoothed, image_consistent_obstacles)
        cv2.namedWindow('Final Combined', cv2.WINDOW_NORMAL)
        cv2.resizeWindow('Final Combined', 1200, 800)
        cv2.imshow('Final Combined', final_image)
        cv2.waitKey(0)
        
        # Save both images
        # cv2.imwrite('original_map.pgm', image)
        # cv2.imwrite('filtered_map.pgm', final_image)
        # print("Saved original_map.pgm and filtered_map.pgm")