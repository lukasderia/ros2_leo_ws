#!/usr/bin/env python3
import rclpy
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import sqlite3
import csv
import os
from pathlib import Path
import matplotlib.pyplot as plt


def extract_frontiers(bag_path, output_dir):
    """Extract frontier markers from ROS2 bag to CSV files"""
    
    # Create output directory
    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    
    # Connect to bag database
    db_path = Path(bag_path) / 'frontiers_bag_0.db3'
    conn = sqlite3.connect(str(db_path))
    cursor = conn.cursor()
    
    # Get topic info
    cursor.execute("SELECT id FROM topics WHERE name='/frontier_markers'")
    topic_id = cursor.fetchone()[0]
    
    # Get message type
    cursor.execute("SELECT type FROM topics WHERE name='/frontier_markers'")
    msg_type = cursor.fetchone()[0]
    
    # Get messages
    cursor.execute("""
        SELECT timestamp, data 
        FROM messages 
        WHERE topic_id=? 
        ORDER BY timestamp
    """, (topic_id,))
    
    messages = cursor.fetchall()
    print(f"Found {len(messages)} messages")
    
    # Get message class
    msg_class = get_message(msg_type)
    
    # Process each message
    for idx, (timestamp, data) in enumerate(messages):
        msg = deserialize_message(data, msg_class)
        
        # Extract points from markers
        points = []
        for marker in msg.markers:
            if marker.type == 2:  # SPHERE type
                points.append([marker.pose.position.x, marker.pose.position.y])
        
        # Write to CSV
        csv_file = output_dir / f"frontiers_{idx:02d}.csv"
        with open(csv_file, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['x', 'y'])
            writer.writerows(points)
        
        print(f"Wrote {len(points)} points to {csv_file.name}")
    
    conn.close()
    print(f"\nExtracted {len(messages)} files to {output_dir}")

def visualize_frontiers(data_dir):
    """Visualize all frontier CSV files"""
    
    data_dir = Path(data_dir)
    csv_files = sorted(data_dir.glob("frontiers_*.csv"))
    
    if not csv_files:
        print(f"No CSV files found in {data_dir}")
        return
    
    print(f"Found {len(csv_files)} files")
    
    for csv_file in csv_files:
        # Read points
        x_coords, y_coords = [], []
        with open(csv_file, 'r') as f:
            reader = csv.DictReader(f)
            for row in reader:
                x_coords.append(float(row['x']))
                y_coords.append(float(row['y']))
        
        # Plot
        plt.figure(figsize=(10, 10))
        plt.scatter(x_coords, y_coords, c='red', s=20)
        plt.axis('equal')
        plt.grid(True)
        plt.title(f"{csv_file.name} - {len(x_coords)} points")
        plt.xlabel('X (meters)')
        plt.ylabel('Y (meters)')
        plt.show()

if __name__ == '__main__':
    # Default path
    bag_path = "/home/lukas/ros2_ws/frontiers_bag"
    output_dir = "/home/lukas/ros2_ws/src/leo_utils/frontier_data"
    
    mode = 1#"extracting"
    if mode == "extracting":
        extract_frontiers(bag_path, output_dir)
    else:
        visualize_frontiers(output_dir)