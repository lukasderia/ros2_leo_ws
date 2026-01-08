import numpy as np
from pathlib import Path
import matplotlib.pyplot as plt

def load_files(data_dir):
    data_dir = Path(data_dir)  # Convert to Path object
    datasets = []

    files = sorted(data_dir.glob("frontiers_*.csv"))

    for file in files:
        #Load files, skip header row
        points = np.loadtxt(file, delimiter=',', skiprows=1)
        datasets.append(points)
        
    print(f"Loaded all files")

    return datasets

def distance_cluster(datasets, threshold=0.4):
    """
    Cluster points where distance < threshold means same cluster
    
    Args:
        datasets: list of numpy arrays, each shape (n, 2)
        threshold: distance threshold in meters (0.1 = 10cm)
    
    Returns:
        labels_list: list of label arrays, one per dataset
    """
    distance_labels = []
    
    for idx, points in enumerate(datasets):
        n = len(points)
        labels = np.full(n, -1)  # -1 means unassigned
        cluster_id = 0
        
        for i in range(n):
            if labels[i] != -1:  # Already assigned
                continue
                
            # Start new cluster with BFS
            queue = [i]
            labels[i] = cluster_id
            
            while queue:
                current = queue.pop(0)
                current_point = points[current]
                
                # Check all other points
                for j in range(n):
                    if labels[j] != -1:  # Already assigned
                        continue
                        
                    # Calculate distance
                    dist = np.linalg.norm(current_point - points[j])
                    
                    if dist < threshold:
                        labels[j] = cluster_id
                        queue.append(j)
            
            cluster_id += 1
        
        print(f"Dataset {idx}: Found {cluster_id} clusters")
        distance_labels.append(labels)
    
    return distance_labels

def generate_distinct_colors(n):
    """Generate n visually distinct colors"""
    import colorsys
    colors = []
    for i in range(n):
        hue = i / n
        # Use full saturation and value for bright, distinct colors
        rgb = colorsys.hsv_to_rgb(hue, 0.8, 0.9)
        colors.append(rgb)
    return colors

def visualize_clustering(datasets, labels_list, method_names):
    """
    Visualize original data and multiple clustering results
    
    Args:
        datasets: list of numpy arrays, each shape (n, 2)
        labels_list: list of label lists, where labels_list[i] contains 
                     labels for all datasets using method i
        method_names: list of strings, names for each method
    """
    n_methods = len(labels_list) + 1  # +1 for original
    
    for idx, dataset in enumerate(datasets):
        fig, axes = plt.subplots(1, n_methods, figsize=(5*n_methods, 5))
        
        # First subplot: original (no clustering)
        axes[0].scatter(dataset[:, 0], dataset[:, 1], c='red', s=20)
        axes[0].set_title('Original')
        axes[0].axis('equal')
        axes[0].grid(True)
        
        # Remaining subplots: clustered versions
        for method_idx, labels in enumerate(labels_list):
            ax = axes[method_idx + 1]
            
            # Generate enough colors for this clustering
            unique_labels = np.unique(labels[idx])
            n_clusters = len(unique_labels)
            colors = generate_distinct_colors(n_clusters)
            
            # Plot points with cluster colors
            ax.scatter(dataset[:, 0], dataset[:, 1], c=labels[idx], s=20, 
                      cmap=plt.cm.colors.ListedColormap(colors))
            
            # Calculate and plot centroids
            for cluster_idx, cluster_label in enumerate(unique_labels):
                # Get all points in this cluster
                cluster_points = dataset[labels[idx] == cluster_label]
                
                # Calculate centroid (mean of all points)
                centroid = cluster_points.mean(axis=0)
                
                # Plot centroid as square
                ax.scatter(centroid[0], centroid[1], c=[colors[cluster_idx]], 
                          s=100, marker='s', edgecolors='black', linewidths=2)
            
            ax.set_title(f'{method_names[method_idx]} ({n_clusters} clusters)')
            ax.axis('equal')
            ax.grid(True)
        
        fig.suptitle(f'Dataset {idx}', fontsize=16)
        plt.tight_layout()
        plt.show()
def main():

    data_dir = "/home/lukas/ros2_ws/src/leo_utils/frontier_data"

    datasets = load_files(data_dir)

    distance_labels = distance_cluster(datasets)

    visualize_clustering(datasets, [distance_labels], ['Distance Cluster'])

if __name__=="__main__":
    main()

