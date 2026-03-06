#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include <cmath>
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "std_msgs/msg/bool.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include <functional>
using std::placeholders::_1;

class TermNode : public rclcpp::Node{
    public:
        TermNode() : Node("TermNode"){
            this->declare_parameter("router_x", 18.0);
            this->declare_parameter("router_y", 18.0);
            router_x_ = this->get_parameter("router_x").as_double();
            router_y_ = this->get_parameter("router_y").as_double();

            map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>("/map_filtered", 10, std::bind(&TermNode::map_callback, this, _1));
            
            tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
            tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

            complete_pub_ = this->create_publisher<std_msgs::msg::Bool>("/exploration_complete", 10);

            check_timer_ = this->create_wall_timer(std::chrono::milliseconds(1000), std::bind(&TermNode::check_termination, this));
        }

    private:
        rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr complete_pub_;
        rclcpp::TimerBase::SharedPtr check_timer_;

        std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

        nav_msgs::msg::OccupancyGrid::SharedPtr latest_map_;

        double robot_x_ = 0.0;
        double robot_y_ = 0.0;
        double router_x_ = 18.0;
        double router_y_ = 18.0;

        void map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg){
            latest_map_ = msg;
        }

        void check_termination(){
            // Get robot position from TF
            try {
                auto transform = tf_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero);
                robot_x_ = transform.transform.translation.x;
                robot_y_ = transform.transform.translation.y;
            } catch (tf2::TransformException &ex) {
                RCLCPP_WARN(this->get_logger(), "Could not get robot position: %s", ex.what());
                return;
            }

            // Check distance to router
            double dx = robot_x_ - router_x_;
            double dy = robot_y_ - router_y_;
            double distance = std::sqrt(dx*dx + dy*dy);

            // Check if router cell is free in map
            bool router_cell_free = isRouterCellFree();

            if (distance < 3.0 && router_cell_free) {
                RCLCPP_INFO(this->get_logger(), "Router found! Distance: %.2f, cell is free", distance);
                std_msgs::msg::Bool msg;
                msg.data = true;
                complete_pub_->publish(msg);
            }
        }

        bool isRouterCellFree(){
            if (!latest_map_) return false;

            // Convert router world position to map cell
            int cell_x = static_cast<int>((router_x_ - latest_map_->info.origin.position.x) / latest_map_->info.resolution);
            int cell_y = static_cast<int>((router_y_ - latest_map_->info.origin.position.y) / latest_map_->info.resolution);

            // Check bounds
            if (cell_x < 0 || cell_x >= (int)latest_map_->info.width ||
                cell_y < 0 || cell_y >= (int)latest_map_->info.height) {
                return false;
            }

            int index = cell_y * latest_map_->info.width + cell_x;
            int8_t cell_value = latest_map_->data[index];

            // Free cell: known and low occupancy probability
            return (cell_value >= 0 && cell_value < 40);
        }
};

int main(int argc, char** argv){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TermNode>());
    rclcpp::shutdown();
    return 0;
}