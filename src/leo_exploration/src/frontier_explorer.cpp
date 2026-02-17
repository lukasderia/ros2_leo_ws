#include <memory>
#include <chrono>
#include <vector>
#include <Eigen/Dense>
#include <algorithm>  // For std::min_element
#include "nav_msgs/msg/odometry.hpp"  // For Odometry message
#include "std_msgs/msg/bool.hpp"  // For Bool message
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include <visualization_msgs/msg/marker.hpp>
#include "leo_exploration/msg/frontier_clusters.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include <sensor_msgs/msg/point_cloud2.hpp>
#include "geometry_msgs/msg/vector3_stamped.hpp"

using std::placeholders::_1;

struct Frontier {
    double x;
    double y;
    int size;
    double distance;
    double heading;
    double score;
    double dx;
    double dy;
};

class FrontierExplorer : public rclcpp::Node{
    public:
    FrontierExplorer() : Node("frontier_explorer"){
        // Declare and get parameter
        this->declare_parameter<std::string>("odom_topic", "/odom");
        std::string odom_topic = this->get_parameter("odom_topic").as_string();

        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        detector_sub_ = this->create_subscription<leo_exploration::msg::FrontierClusters>("/frontier_centroids", 10, std::bind(&FrontierExplorer::detector_callback, this, _1));

        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(odom_topic,10, std::bind(&FrontierExplorer::odom_callback, this, _1));
        
        mode_sub_ = this->create_subscription<std_msgs::msg::Bool>("/auto_mode", rclcpp::QoS(10).transient_local(), std::bind(&FrontierExplorer::mode_callback, this, _1));

        explorer_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/goal_pose", 10);

        gradient_sub_ = this->create_subscription<geometry_msgs::msg::Vector3Stamped>("/rss_gradient", 10,std::bind(&FrontierExplorer::gradient_callback, this, _1));

    }

    private:
        rclcpp::Subscription<leo_exploration::msg::FrontierClusters>::SharedPtr detector_sub_; // Subscriber for detector
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_; // Subscriber for robot pose
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr mode_sub_; //Subscriber for auto_mode
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr explorer_pub_; // Publisher for nav2
        rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr gradient_sub_;


        std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

        leo_exploration::msg::FrontierClusters::SharedPtr latest_centroids_;
        nav_msgs::msg::Odometry::SharedPtr latest_odom_;
        bool auto_mode_enabled_ = false;
        double robot_yaw_, robot_y_, robot_x_;
        bool odom_recieved_ = false;

        std::vector<Frontier> frontier_list_;

        // In private section
        bool has_published_goal_ = false;
        double last_goal_x_ = 0.0;
        double last_goal_y_ = 0.0;

        double latest_grad_x_ = 0.0;
        double latest_grad_y_ = 0.0;
        bool gradient_received_ = false;

        double max_size_ = 0.0;
        double min_size_ = 0.0;
        double max_dist_ = 0.0;
        double min_dist_ = 0.0;

        void detector_callback(const leo_exploration::msg::FrontierClusters::SharedPtr msg){
            latest_centroids_ = msg;
            frontier_list_.clear();

            // Check if we have odometry data
            if (!odom_recieved_) {
                RCLCPP_WARN(this->get_logger(), "No odometry data available");
                return;
            }
            
            // Check if autonomous mode is enabled
            if (!auto_mode_enabled_) {
                return;  // Not in autonomous mode, skip processing
            }
            
            // transform odom msg from odom frame to map frame
            try {
                // Get transform from odom to map
                geometry_msgs::msg::TransformStamped transform_stamped = tf_buffer_->lookupTransform("map", "odom", tf2::TimePointZero);

                // Transform robot pose from odom to map
                geometry_msgs::msg::PoseStamped pose_odom, pose_map;
                pose_odom.header = latest_odom_->header;
                pose_odom.pose = latest_odom_->pose.pose;

                tf2::doTransform(pose_odom, pose_map, transform_stamped);

                // extract robot pose 
                robot_x_ = pose_map.pose.position.x;
                robot_y_ = pose_map.pose.position.y;
                auto q = pose_map.pose.orientation;
                robot_yaw_ = atan2(2.0*(q.y*q.z + q.w*q.x), q.w*q.w - q.x*q.x - q.y*q.y + q.z*q.z);

            } catch (tf2::TransformException &ex) {
                RCLCPP_WARN(this->get_logger(), "Could not transform: %s", ex.what());
                return;
            }

            // Use gradient from RSS node (if available)
            if (gradient_received_) {
                RCLCPP_INFO(this->get_logger(), "Using RSS Gradient: [%.3f, %.3f]", 
                            latest_grad_x_, latest_grad_y_);
            }

            min_size_ = std::numeric_limits<double>::max();  // Start at huge value
            max_size_ = std::numeric_limits<double>::lowest();  // Start at tiny value
            min_dist_ = std::numeric_limits<double>::max();
            max_dist_ = std::numeric_limits<double>::lowest();

            // loop through and assign the data to the list and calculate the metrics as well as normalizing
            for (const auto& cluster : latest_centroids_->clusters){ 
                Frontier f;
                f.x = cluster.x;
                f.y = cluster.y;
                f.size = cluster.size;

                min_size_ = std::min(min_size_, static_cast<double>(f.size));
                max_size_ = std::max(max_size_, static_cast<double>(f.size));

                // Calculate distance
                f.dx = f.x - robot_x_;
                f.dy = f.y - robot_y_;

                f.distance = std::sqrt(f.dx*f.dx + f.dy*f.dy);
                
                min_dist_ = std::min(min_dist_, f.distance);
                max_dist_ = std::max(max_dist_, f.distance);

                // Calculate heading 
                double bearing = atan2(f.dy, f.dx);
                f.heading = bearing - robot_yaw_;

                frontier_list_.push_back(f);
            }
            
            for (auto& f : frontier_list_) {
                f.score = calculate_score(f, max_dist_, min_dist_, max_size_, min_size_);
            }

            // After populating frontier_list_
            if (frontier_list_.empty()) {
                RCLCPP_WARN(this->get_logger(), "No frontiers to explore");
                return;
            }

            // Find frontier with minimum score
            auto best = std::max_element(frontier_list_.begin(), frontier_list_.end(),
                [](const Frontier& a, const Frontier& b) {
                    return a.score < b.score;
                });

            // Check if new goal is significantly different from last goal
            if (has_published_goal_) {
                double dx = best->x - last_goal_x_;
                double dy = best->y - last_goal_y_;
                double distance_to_last_goal = std::sqrt(dx*dx + dy*dy);
                
                double threshold = 0.50;  // meters
                
                if (distance_to_last_goal < threshold) {
                    RCLCPP_DEBUG(this->get_logger(), "New goal too close to last goal (%.2f m), skipping", distance_to_last_goal);
                    return;
                }
            }

            // Publish the best frontier
            publish_goal(*best);
        }

        void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg){
            latest_odom_ = msg;
            odom_recieved_ = true;
        }

        void mode_callback (const std_msgs::msg::Bool::SharedPtr msg){
            bool previous_mode = auto_mode_enabled_;
            auto_mode_enabled_ = msg->data;
            
            // If switching from auto to manual (true -> false)
            if (previous_mode && !auto_mode_enabled_) {
                stop_robot();
                has_published_goal_ = false;
            }
        }

        void gradient_callback(const geometry_msgs::msg::Vector3Stamped::SharedPtr msg) {
            latest_grad_x_ = msg->vector.x;
            latest_grad_y_ = msg->vector.y;
            gradient_received_ = true;
            RCLCPP_DEBUG(this->get_logger(), "Received gradient: [%.3f, %.3f]", 
                        latest_grad_x_, latest_grad_y_);
        }

        void stop_robot() {
            if (!odom_recieved_) {
                RCLCPP_WARN(this->get_logger(), "Cannot stop - no odometry data");
                return;
            }
            
            try {
                // Get transform from odom to map
                geometry_msgs::msg::TransformStamped transform_stamped = 
                    tf_buffer_->lookupTransform("map", "odom", tf2::TimePointZero);

                // Transform current robot pose to map frame
                geometry_msgs::msg::PoseStamped pose_odom, pose_map;
                pose_odom.header = latest_odom_->header;
                pose_odom.pose = latest_odom_->pose.pose;

                tf2::doTransform(pose_odom, pose_map, transform_stamped);

                // Publish current position as goal
                geometry_msgs::msg::PoseStamped stop_goal;
                stop_goal.header.stamp = this->now();
                stop_goal.header.frame_id = "map";
                stop_goal.pose = pose_map.pose;
                
                explorer_pub_->publish(stop_goal);
                RCLCPP_INFO(this->get_logger(), "Auto mode OFF - robot stopping at current position");
                
            } catch (tf2::TransformException &ex) {
                RCLCPP_WARN(this->get_logger(), "Failed to stop robot: %s", ex.what());
            }
        }

        void publish_goal(const Frontier& frontier) {
            geometry_msgs::msg::PoseStamped goal;
            goal.header.stamp = this->now();
            goal.header.frame_id = "map";
            goal.pose.position.x = frontier.x;
            goal.pose.position.y = frontier.y;
            goal.pose.position.z = 0.0;
            goal.pose.orientation.w = 1.0;  // Neutral orientation
            
            explorer_pub_->publish(goal);
            
            // Store this goal for future comparison
            last_goal_x_ = frontier.x;
            last_goal_y_ = frontier.y;
            has_published_goal_ = true;
            
            RCLCPP_INFO(this->get_logger(), "Published goal: (%.2f, %.2f) with score %.2f", 
                        frontier.x, frontier.y, frontier.score);
        }

        double calculate_score(const Frontier& f, double max_dist, double min_dist, double max_size, double min_size){
            // Weights
            double w_distance = 0;
            double w_heading = 0;
            double w_size = 0;
            double w_gradient = gradient_received_ ? 1 : 0;

            double norm_distance = (max_dist == min_dist) ? 0.5 : (f.distance - min_dist) / (max_dist - min_dist);
            double norm_size = (max_size == min_size) ? 0.5 : (f.size - min_size) / (max_size - min_size);
            double norm_heading = std::abs(f.heading) / M_PI;  // heading is 0 to π

            // RSS gradient alignment
            double norm_gradient = 0.5;
            if (gradient_received_) {
                double gradient_bearing = atan2(latest_grad_y_, latest_grad_x_);
                double frontier_bearing = atan2(f.dy, f.dx);  // Use stored values
                double angle_diff = std::abs(gradient_bearing - frontier_bearing);
                
                if (angle_diff > M_PI) angle_diff = 2*M_PI - angle_diff;
                norm_gradient = angle_diff / M_PI;
            }

            // Invert so 1.0 = best
            double inv_norm_distance = 1.0 - norm_distance;
            double inv_norm_heading = 1.0 - norm_heading;
            double inv_norm_gradient = 1.0 - norm_gradient;

            double score = w_distance * inv_norm_distance + w_size * norm_size + w_heading * inv_norm_heading + w_gradient * inv_norm_gradient;
            
            return score;
            }
    };

int main(int argc, char** argv){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FrontierExplorer>());
    rclcpp::shutdown();
    return 0;
}