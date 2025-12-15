#include <memory>
#include <chrono>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

using std::placeholders::_1;

struct Point {
    int x;
    int y;
};

class FrontierDetector : public rclcpp::Node{
    public:
        FrontierDetector() : Node("frontier_detector"){
            map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
                "/map", 10, std::bind(&FrontierDetector::map_callback, this, _1));

            marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/frontier_markers", 10);

            timer_ = this->create_wall_timer(std::chrono::seconds(2), std::bind(&FrontierDetector::detect_frontiers, this));
        }

    private:
        rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
        nav_msgs::msg::OccupancyGrid::SharedPtr latest_map_;
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
        rclcpp::TimerBase::SharedPtr timer_;
        
        void map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg){
            // Store the map, process it later
            latest_map_ = msg;
        }

        void detect_frontiers(){
            if (!latest_map_) return;  // No map yet
            
            int width = latest_map_->info.width;
            int height = latest_map_->info.height;

            std::vector<Point> frontier_cells;
            
            // Your frontier detection loop goes here
            for(int i = 0; i < width*height; i++){
                int8_t cell_value = latest_map_->data[i];
                
                if(cell_value >= 0 && cell_value < 40){  // Cell is free
                    int y = i / width;
                    int x = i % width;
                    
                    bool is_frontier = false;
                    
                    // Check all 4 neighbors
                    // Right
                    if(x < width-1 && latest_map_->data[i+1] == -1) is_frontier = true;
                    // Left
                    if(x > 0 && latest_map_->data[i-1] == -1) is_frontier = true;
                    // Up
                    if(y < height-1 && latest_map_->data[i+width] == -1) is_frontier = true;
                    // Down
                    if(y > 0 && latest_map_->data[i-width] == -1) is_frontier = true;
                    
                    if(is_frontier){
                        // Store this cell's coordinates (x,y) in a list
                        frontier_cells.push_back({x,y});
                    }
                }
            }
            
            // After your frontier detection loop
            visualization_msgs::msg::MarkerArray marker_array;

            for (size_t i = 0; i < frontier_cells.size(); i++) {
                visualization_msgs::msg::Marker marker;
                marker.header.frame_id = latest_map_->header.frame_id;
                marker.header.stamp = this->now();
                marker.ns = "frontiers";
                marker.id = i;
                marker.type = visualization_msgs::msg::Marker::SPHERE;
                marker.action = visualization_msgs::msg::Marker::ADD;
                
                // Convert grid coordinates to world coordinates
                marker.pose.position.x = latest_map_->info.origin.position.x + 
                                        (frontier_cells[i].x + 0.5) * latest_map_->info.resolution;
                marker.pose.position.y = latest_map_->info.origin.position.y + 
                                        (frontier_cells[i].y + 0.5) * latest_map_->info.resolution;
                marker.pose.position.z = 0.0;
                
                marker.scale.x = 0.1;
                marker.scale.y = 0.1;
                marker.scale.z = 0.1;
                
                marker.color.r = 0.50;
                marker.color.g = 0.0;
                marker.color.b = 0.50;
                marker.color.a = 1.0;
                
                marker_array.markers.push_back(marker);
            }

            marker_pub_->publish(marker_array);

        }
};


int main(int argc, char** argv){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FrontierDetector>());
    rclcpp::shutdown();
    return 0;
}