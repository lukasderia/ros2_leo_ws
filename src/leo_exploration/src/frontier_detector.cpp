#include <memory>
#include <chrono>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "leo_exploration/msg/frontier_clusters.hpp"

using std::placeholders::_1;

struct Point {
    int x;
    int y;
};

struct WorldPoint {
    double x;
    double y;
}

struct Cluster {
    WorldPoint centroid;  // World coordinates
    int size;        // Number of points in cluster
    int id;          // Cluster ID
};

class FrontierDetector : public rclcpp::Node{
    public:
        FrontierDetector() : Node("frontier_detector"){
            map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
                "/map_filtered", 10, std::bind(&FrontierDetector::map_callback, this, _1));

            marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/frontier_markers", 10);

            centroid_pub_ = this->create_publisher<leo_exploration::msg::FrontierClusters>("/frontier_centroids", 10);

            centroid_marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/frontier_centroid_markers", 10);
        }

    private:
        rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
        nav_msgs::msg::OccupancyGrid::SharedPtr latest_map_;
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
        rclcpp::Publisher<leo_exploration::msg::FrontierClusters>::SharedPtr centroid_pub_;
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr centroid_marker_pub_;

        // Member variable to store frontier cells after detection
        std::vector<Point> frontier_cells_;
        std::vector<Cluster> clusters_;
        std::vector<int> frontier_labels_;
        
        void map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg){
            // Store the map, process it later
            latest_map_ = msg;

            detect_frontiers();
            // After detection, run the full pipeline:
            cluster_frontiers();       // Cluster the detected frontiers
            publish_frontier_markers();   // Visualize all frontier cells
            publish_centroids();       // Publish cluster centroids
            publish_centroid_markers(); //Publish the centroids as points to visualize
        }

        void detect_frontiers() {
            if (!latest_map_) return;
            
            frontier_cells_.clear();  // Clear previous detection
            
            int width = latest_map_->info.width;
            int height = latest_map_->info.height;
            
            for(int i = 0; i < width*height; i++){
                int8_t cell_value = latest_map_->data[i];
                
                if(cell_value >= 0 && cell_value < 40){  // Cell is free
                    int y = i / width;
                    int x = i % width;
                    
                    bool is_frontier = false;
                    
                    // Check all 4 neighbors
                    if(x < width-1 && latest_map_->data[i+1] == -1) is_frontier = true;
                    if(x > 0 && latest_map_->data[i-1] == -1) is_frontier = true;
                    if(y < height-1 && latest_map_->data[i+width] == -1) is_frontier = true;
                    if(y > 0 && latest_map_->data[i-width] == -1) is_frontier = true;
                    
                    if(is_frontier){
                        frontier_cells_.push_back({x, y});
                    }
                }
            }
        }
        
        void cluster_frontiers() {
            clusters_.clear();
            
            if (frontier_cells_.empty()) return;
            
            // Convert grid coordinates to world coordinates for clustering
            std::vector<WorldPoint> world_points;
            for (const auto& cell : frontier_cells_) {
                WorldPoint world;
                world.x = latest_map_->info.origin.position.x + 
                        (cell.x + 0.5) * latest_map_->info.resolution;
                world.y = latest_map_->info.origin.position.y + 
                        (cell.y + 0.5) * latest_map_->info.resolution;
                world_points.push_back(world);
            }
            
            // Distance-based clustering (BFS approach)
            const double threshold = 0.3;  // distance threshold
            std::vector<int> labels(world_points.size(), -1);
            int cluster_id = 0;
            
            for (size_t i = 0; i < world_points.size(); i++) {
                if (labels[i] != -1) continue;  // Already assigned
                
                // Start new cluster with BFS
                std::vector<size_t> queue;
                queue.push_back(i);
                labels[i] = cluster_id;
                
                size_t idx = 0;
                while (idx < queue.size()) {
                    size_t current = queue[idx++];
                    
                    for (size_t j = 0; j < world_points.size(); j++) {
                        if (labels[j] != -1) continue;
                        
                        double dx = world_points[current].x - world_points[j].x;
                        double dy = world_points[current].y - world_points[j].y;
                        double dist = std::sqrt(dx*dx + dy*dy);
                        
                        if (dist < threshold) {
                            labels[j] = cluster_id;
                            queue.push_back(j);
                        }
                    }
                }
                cluster_id++;
            }
            
            // Calculate centroid and size for each cluster
            for (int id = 0; id < cluster_id; id++) {
                double sum_x = 0, sum_y = 0;
                int count = 0;
                
                for (size_t i = 0; i < world_points.size(); i++) {
                    if (labels[i] == id) {
                        sum_x += world_points[i].x;
                        sum_y += world_points[i].y;
                        count++;
                    }
                }
                
                if (count > 0) {
                    Cluster cluster;
                    cluster.centroid.x = sum_x / count;
                    cluster.centroid.y = sum_y / count;
                    cluster.size = count;
                    cluster.id = id;
                    clusters_.push_back(cluster);
                }
            }
            
            frontier_labels_ = labels;
            RCLCPP_INFO(this->get_logger(), "Found %zu clusters", clusters_.size());
        }
        
        void publish_frontier_markers() {
            if (!latest_map_ || frontier_cells_.empty()) return;
            
            visualization_msgs::msg::MarkerArray marker_array;
            
            for (size_t i = 0; i < frontier_cells_.size(); i++) {
                visualization_msgs::msg::Marker marker;
                marker.header.frame_id = latest_map_->header.frame_id;
                marker.header.stamp = this->now();
                marker.ns = "frontiers";
                marker.id = i;
                marker.action = visualization_msgs::msg::Marker::ADD;
                
                // Convert grid coordinates to world coordinates
                marker.pose.position.x = latest_map_->info.origin.position.x + 
                                        (frontier_cells_[i].x + 0.5) * latest_map_->info.resolution;
                marker.pose.position.y = latest_map_->info.origin.position.y + 
                                        (frontier_cells_[i].y + 0.5) * latest_map_->info.resolution;
                marker.pose.position.z = 0.0;

                marker.scale.x = 0.1;
                marker.scale.y = 0.1;
                marker.scale.z = 0.1;
                
                // Get cluster ID for this frontier cell
                int cluster_id = frontier_labels_[i];

                // Determine color (10 colors)
                int color_index = cluster_id % 10;
                switch(color_index) {
                    case 0: // Red
                        marker.color.r = 1.0; marker.color.g = 0.0; marker.color.b = 0.0;
                        break;
                    case 1: // Green
                        marker.color.r = 0.0; marker.color.g = 1.0; marker.color.b = 0.0;
                        break;
                    case 2: // Blue
                        marker.color.r = 0.0; marker.color.g = 0.0; marker.color.b = 1.0;
                        break;
                    case 3: // Yellow
                        marker.color.r = 1.0; marker.color.g = 1.0; marker.color.b = 0.0;
                        break;
                    case 4: // Cyan
                        marker.color.r = 0.0; marker.color.g = 1.0; marker.color.b = 1.0;
                        break;
                    case 5: // Magenta
                        marker.color.r = 1.0; marker.color.g = 0.0; marker.color.b = 1.0;
                        break;
                    case 6: // Orange
                        marker.color.r = 1.0; marker.color.g = 0.5; marker.color.b = 0.0;
                        break;
                    case 7: // White
                        marker.color.r = 1.0; marker.color.g = 1.0; marker.color.b = 1.0;
                        break;
                    case 8: // Lime
                        marker.color.r = 0.5; marker.color.g = 1.0; marker.color.b = 0.0;
                        break;
                    case 9: // Pink
                        marker.color.r = 1.0; marker.color.g = 0.4; marker.color.b = 0.7;
                        break;
                }
                marker.color.a = 1.0;

                // Determine shape (3 shapes)
                int shape_index = cluster_id / 10;
                switch(shape_index) {
                    case 0:
                        marker.type = visualization_msgs::msg::Marker::SPHERE;
                        break;
                    case 1:
                        marker.type = visualization_msgs::msg::Marker::CUBE;
                        break;
                    case 2:
                        marker.type = visualization_msgs::msg::Marker::CYLINDER;
                        break;
                    default:
                        marker.type = visualization_msgs::msg::Marker::SPHERE;
                        break;
                }
                marker_array.markers.push_back(marker);
            }
            
            marker_pub_->publish(marker_array);
        }
        
        void publish_centroids() {
            if (clusters_.empty()) return;

            // Create message using user defined template
            leo_exploration::msg::FrontierClusters msg;
            msg.header.stamp = this->now();
            msg.header.frame_id = latest_map_->header.frame_id;

            for (const auto& cluster : clusters_){
                leo_exploration::msg::FrontierCluster cluster_msg;
                cluster_msg.x = cluster.centroid.x;
                cluster_msg.y = cluster.centroid.y;
                cluster_msg.size = cluster.size;

                msg.clusters.push_back(cluster_msg);
            }

            centroid_pub_->publish(msg);
        }

        void publish_centroid_markers(){
            if(clusters_.empty()) return;
            
            // Create message and iunitiate with time and frame from map
            visualization_msgs::msg::MarkerArray centroid_marker_array;

            for (size_t i = 0; i < clusters_.size(); i++){
                visualization_msgs::msg::Marker centroid_marker;
                centroid_marker.header.stamp = this->now();
                centroid_marker.header.frame_id = latest_map_->header.frame_id;
                centroid_marker.ns = "centroids";
                centroid_marker.id = i;
                centroid_marker.type = visualization_msgs::msg::Marker::SPHERE;
                centroid_marker.action = visualization_msgs::msg::Marker::ADD;

                centroid_marker.pose.position.x = clusters_[i].centroid.x;
                centroid_marker.pose.position.y = clusters_[i].centroid.y;
                centroid_marker.pose.position.z = 0.0;
                
                centroid_marker.scale.x = 0.1;
                centroid_marker.scale.y = 0.1;
                centroid_marker.scale.z = 0.1;
                
                centroid_marker.color.r = 0.50;
                centroid_marker.color.g = 0.0;
                centroid_marker.color.b = 0.50;
                centroid_marker.color.a = 1.0;
                
                centroid_marker_array.markers.push_back(centroid_marker);
            }

            centroid_marker_pub_->publish(centroid_marker_array);
        }
    };


int main(int argc, char** argv){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FrontierDetector>());
    rclcpp::shutdown();
    return 0;
}