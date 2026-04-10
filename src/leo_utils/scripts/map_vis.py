#!/usr/bin/env python3
"""
Plot 3 panels: raw map + robot, filtered map + robot, filtered map + robot + frontiers.
Usage: python3 plot_frontiers.py <path_to_bag_directory>
"""

import sys
import os
import sqlite3
import numpy as np
import matplotlib.pyplot as plt
from rclpy.serialization import deserialize_message
from nav_msgs.msg import OccupancyGrid, Odometry
from visualization_msgs.msg import MarkerArray
import tf2_ros
from tf2_msgs.msg import TFMessage
import math


def read_messages(bag_path, topic, msg_type):
    """Read all messages for a given topic from a rosbag."""
    db_files = [f for f in os.listdir(bag_path) if f.endswith('.db3')]
    if not db_files:
        print(f"No .db3 file found in {bag_path}")
        sys.exit(1)

    db_path = os.path.join(bag_path, db_files[0])
    conn = sqlite3.connect(db_path)
    cursor = conn.cursor()

    cursor.execute("SELECT id FROM topics WHERE name = ?", (topic,))
    row = cursor.fetchone()
    if row is None:
        print(f"Topic {topic} not found in bag")
        conn.close()
        return []

    topic_id = row[0]

    cursor.execute(
        "SELECT timestamp, data FROM messages WHERE topic_id = ? ORDER BY timestamp ASC",
        (topic_id,)
    )

    messages = []
    for timestamp, data in cursor.fetchall():
        msg = deserialize_message(data, msg_type)
        messages.append((timestamp, msg))

    conn.close()
    return messages


def map_to_image(msg):
    """Convert OccupancyGrid message to a plottable numpy array."""
    width = msg.info.width
    height = msg.info.height
    data = np.array(msg.data).reshape((height, width))

    image = np.full((height, width, 3), 0.5)  # grey = unknown
    image[data == 0] = [1.0, 1.0, 1.0]        # white = free
    image[data == 100] = [0.0, 0.0, 0.0]      # black = occupied

    mask = (data > 0) & (data < 100)
    if np.any(mask):
        image[mask] = np.stack([1.0 - data[mask] / 100.0] * 3, axis=-1)


    return image


def get_map_extent(msg):
    """Get the world coordinate extent of the map."""
    origin_x = msg.info.origin.position.x
    origin_y = msg.info.origin.position.y
    res = msg.info.resolution
    width = msg.info.width
    height = msg.info.height

    x_min = origin_x
    x_max = origin_x + width * res
    y_min = origin_y
    y_max = origin_y + height * res

    return [x_min, x_max, y_min, y_max]


def quaternion_to_yaw(q):
    """Extract yaw from quaternion."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def get_map_to_odom_transform(bag_path):
    """Extract the last map->odom transform from /tf messages."""
    tf_msgs = read_messages(bag_path, "/tf", TFMessage)
    if not tf_msgs:
        print("No /tf messages found")
        return None

    # Walk backwards to find the latest map->odom transform
    for timestamp, msg in reversed(tf_msgs):
        for transform in msg.transforms:
            if transform.header.frame_id == "map" and transform.child_frame_id == "odom":
                return transform.transform

    print("No map->odom transform found in /tf")
    return None


def apply_transform(tf, x, y, yaw):
    """Apply a TF transform to a 2D pose."""
    # Extract transform components
    tx = tf.translation.x
    ty = tf.translation.y
    tf_yaw = quaternion_to_yaw(tf.rotation)

    # Rotate then translate
    new_x = math.cos(tf_yaw) * x - math.sin(tf_yaw) * y + tx
    new_y = math.sin(tf_yaw) * x + math.cos(tf_yaw) * y + ty
    new_yaw = yaw + tf_yaw

    return new_x, new_y, new_yaw


def get_robot_position(bag_path):
    """Get the last robot position in map frame by transforming odom through TF."""
    odom_msgs = read_messages(bag_path, "/odom", Odometry)
    if not odom_msgs:
        print("No /odom messages found")
        return None, None, None

    last_odom = odom_msgs[-1][1]
    odom_x = last_odom.pose.pose.position.x
    odom_y = last_odom.pose.pose.position.y
    odom_yaw = quaternion_to_yaw(last_odom.pose.pose.orientation)

    # Get map->odom transform
    tf = get_map_to_odom_transform(bag_path)
    if tf is None:
        print("  Warning: no transform found, using raw odom")
        return odom_x, odom_y, odom_yaw

    map_x, map_y, map_yaw = apply_transform(tf, odom_x, odom_y, odom_yaw)
    print(f"  Odom: ({odom_x:.2f}, {odom_y:.2f}) -> Map: ({map_x:.2f}, {map_y:.2f})")
    return map_x, map_y, map_yaw


def get_frontier_positions(bag_path):
    """Get frontier centroid positions from the last /frontier_centroid_markers message."""
    marker_msgs = read_messages(bag_path, "/frontier_centroid_markers", MarkerArray)
    if not marker_msgs:
        print("No /frontier_centroid_markers messages found")
        return [], []

    last_markers = marker_msgs[-1][1]
    fx, fy = [], []
    for marker in last_markers.markers:
        fx.append(marker.pose.position.x)
        fy.append(marker.pose.position.y)

    return fx, fy


def get_frontier_cell_positions(bag_path):
    """Get all frontier cell positions from the last /frontier_markers message."""
    marker_msgs = read_messages(bag_path, "/frontier_markers", MarkerArray)
    if not marker_msgs:
        print("No /frontier_markers messages found")
        return [], []

    last_markers = marker_msgs[-1][1]
    fx, fy = [], []
    for marker in last_markers.markers:
        fx.append(marker.pose.position.x)
        fy.append(marker.pose.position.y)

    return fx, fy


def main():
    if len(sys.argv) < 2:
        print("Usage: python3 plot_frontiers.py <path_to_bag_directory>")
        sys.exit(1)

    bag_path = sys.argv[1]

    print("Reading /map messages...")
    map_msgs = read_messages(bag_path, "/map", OccupancyGrid)
    print(f"  Found {len(map_msgs)} messages")

    print("Reading /map_filtered messages...")
    filtered_msgs = read_messages(bag_path, "/map_filtered", OccupancyGrid)
    print(f"  Found {len(filtered_msgs)} messages")

    print("Reading robot position...")
    robot_x, robot_y, robot_yaw = get_robot_position(bag_path)
    if robot_x is not None:
        print(f"  Robot in map frame at ({robot_x:.2f}, {robot_y:.2f}), yaw={math.degrees(robot_yaw):.1f}°")

    print("Reading frontier centroids...")
    centroid_x, centroid_y = get_frontier_positions(bag_path)
    print(f"  Found {len(centroid_x)} centroids")

    print("Reading frontier cells...")
    cell_x, cell_y = get_frontier_cell_positions(bag_path)
    print(f"  Found {len(cell_x)} frontier cells")

    # Use last messages
    raw_map = map_msgs[-1][1]
    filtered_map = filtered_msgs[-1][1]

    raw_img = map_to_image(raw_map)
    filtered_img = map_to_image(filtered_map)

    raw_extent = get_map_extent(raw_map)
    filtered_extent = get_map_extent(filtered_map)

    def plot_robot(ax, x, y):
        """Draw robot as a red square."""
        ax.plot(x, y, 's', color="#CC0000", markersize=14, markeredgecolor='#660000',
                markeredgewidth=2, zorder=5, label='Robot')

    # Plot 1x3 grid
    fig, axes = plt.subplots(1, 3, figsize=(21, 7))

    # Panel 1: Raw map + robot
    axes[0].imshow(raw_img, extent=raw_extent, origin='lower')
    if robot_x is not None:
        plot_robot(axes[0], robot_x, robot_y)
    axes[0].set_title("Raw Map", fontsize=14)
    axes[0].set_xlabel("x (m)")
    axes[0].set_ylabel("y (m)")

    # Panel 2: Filtered map + robot
    axes[1].imshow(filtered_img, extent=filtered_extent, origin='lower')
    if robot_x is not None:
        plot_robot(axes[1], robot_x, robot_y)
    axes[1].set_title("Filtered Map", fontsize=14)
    axes[1].set_xlabel("x (m)")
    axes[1].set_ylabel("y (m)")

    # Panel 3: Filtered map + robot + frontiers
    axes[2].imshow(filtered_img, extent=filtered_extent, origin='lower')
    if robot_x is not None:
        plot_robot(axes[2], robot_x, robot_y)
    if cell_x:
        axes[2].scatter(cell_x, cell_y, c='red', s=20, alpha=0.7, label='Frontier cells', zorder=3)
    if centroid_x:
        axes[2].scatter(centroid_x, centroid_y, c='black', s=160, marker='x',
                        linewidths=2.5, label='Centroids', zorder=4)
    axes[2].set_title("Filtered Map + Frontiers", fontsize=14)
    axes[2].set_xlabel("x (m)")
    axes[2].set_ylabel("y (m)")
    # Force same axis limits on all panels
    for ax in axes:
        ax.set_xlim(filtered_extent[0], filtered_extent[1])
        ax.set_ylim(filtered_extent[2], filtered_extent[3])

    plt.tight_layout()
    plt.savefig("frontier_detection.pdf", dpi=300, bbox_inches='tight')
    plt.show()

    print("\nSaved: frontier_detection.pdf")


if __name__ == "__main__":
    main()


# #!/usr/bin/env python3
# """
# Plot /map and /map_filtered from a rosbag side by side.
# Usage: python3 plot_maps.py <path_to_bag_directory>
# """

# import sys
# import sqlite3
# import numpy as np
# import matplotlib.pyplot as plt
# from rclpy.serialization import deserialize_message
# from nav_msgs.msg import OccupancyGrid

# def read_messages(bag_path, topic):
#     """Read all messages for a given topic from a rosbag."""
#     db_files = [f for f in __import__('os').listdir(bag_path) if f.endswith('.db3')]
#     if not db_files:
#         print(f"No .db3 file found in {bag_path}")
#         sys.exit(1)
    
#     db_path = __import__('os').path.join(bag_path, db_files[0])
#     conn = sqlite3.connect(db_path)
#     cursor = conn.cursor()
    
#     # Get topic id
#     cursor.execute("SELECT id FROM topics WHERE name = ?", (topic,))
#     row = cursor.fetchone()
#     if row is None:
#         print(f"Topic {topic} not found in bag")
#         conn.close()
#         return []
    
#     topic_id = row[0]
    
#     # Get all messages for this topic
#     cursor.execute(
#         "SELECT timestamp, data FROM messages WHERE topic_id = ? ORDER BY timestamp ASC",
#         (topic_id,)
#     )
    
#     messages = []
#     for timestamp, data in cursor.fetchall():
#         msg = deserialize_message(data, OccupancyGrid)
#         messages.append((timestamp, msg))
    
#     conn.close()
#     return messages


# def map_to_image(msg):
#     """Convert OccupancyGrid message to a plottable numpy array."""
#     width = msg.info.width
#     height = msg.info.height
#     data = np.array(msg.data).reshape((height, width))
    
#     # Create RGB image: free=white, occupied=black, unknown=grey
#     image = np.full((height, width, 3), 0.5)  # grey = unknown (-1)
#     image[data == 0] = [1.0, 1.0, 1.0]        # white = free
#     image[data == 100] = [0.0, 0.0, 0.0]      # black = occupied
    
#     # For values between 1-99, scale from white to black
#     mask = (data > 0) & (data < 100)
#     if np.any(mask):
#         image[mask] = np.stack([1.0 - data[mask]/100.0]*3, axis=-1)
    
#     # Flip vertically so origin is at bottom-left
#     image = np.flipud(image)
    
#     # Crop: remove top 25% and right 10%
#     h, w = image.shape[:2]
#     top_crop = int(h * 0.25)
#     right_crop = int(w * 0.90)
#     image = image[top_crop:, :right_crop]
    
#     return image


# def main():
#     if len(sys.argv) < 2:
#         print("Usage: python3 plot_maps.py <path_to_bag_directory>")
#         sys.exit(1)
    
#     bag_path = sys.argv[1]
    
#     print("Reading /map messages...")
#     map_msgs = read_messages(bag_path, "/map")
#     print(f"  Found {len(map_msgs)} messages")
    
#     print("Reading /map_filtered messages...")
#     filtered_msgs = read_messages(bag_path, "/map_filtered")
#     print(f"  Found {len(filtered_msgs)} messages")
    
#     # # Plot first message from each side by side
#     n = min(len(map_msgs), len(filtered_msgs))
    
#     # # Show first pair
#     # fig, axes = plt.subplots(1, 2, figsize=(14, 6))
    
#     # map_img = map_to_image(map_msgs[0][1])
#     # filtered_img = map_to_image(filtered_msgs[0][1])
    
#     # axes[0].imshow(map_img)
#     # axes[0].set_title(f"/map - message 0")
#     # axes[0].axis('off')
    
#     # axes[1].imshow(filtered_img)
#     # axes[1].set_title(f"/map_filtered - message 0")
#     # axes[1].axis('off')
    
#     # plt.suptitle("First messages - check alignment")
#     # plt.tight_layout()
#     # plt.savefig("maps_first_pair.png", dpi=150, bbox_inches='tight')
#     # plt.show()
    
#     # Show last 4 pairs
#     print(f"\nTotal pairs available: {n}")
#     print(f"Showing last 4 pairs (indices {n-4} to {n-1})")
        
#     fig, axes = plt.subplots(2, 3, figsize=(15, 10))

#     for i, idx in enumerate(range(n-12, n-3, 3)):
#         map_img = map_to_image(map_msgs[idx][1])
#         filtered_img = map_to_image(filtered_msgs[idx][1])
        
#         axes[0, i].imshow(map_img)
#         axes[0, i].set_title(f"/map [{idx}]")
#         axes[0, i].axis('off')
        
#         axes[1, i].imshow(filtered_img)
#         axes[1, i].set_title(f"/map_filtered [{idx}]")
#         axes[1, i].axis('off')
    
#     plt.tight_layout()
#     plt.savefig("maps_last4_pairs.pdf", dpi=300, bbox_inches='tight')
#     plt.show()
    
#     print("\nSaved: maps_first_pair.png, maps_last4_pairs.png")


# if __name__ == "__main__":
#     main()