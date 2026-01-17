#include <memory>
#include <chrono>
#include <vector>
#include <algorithm>  // For std::min_element
#include "nav_msgs/msg/odometry.hpp"  // For Odometry message
#include "std_msgs/msg/bool.hpp"  // For Bool message
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "leo_exploration/msg/frontier_clusters.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

using std::placeholders::_1;

struct Frontier {
    double x;
    double y;
    int size;
    double distance;
    double heading;
    double score;
};

class FrontierExplorer : public rclcpp::Node{
    public:
    FrontierExplorer() : Node("frontier_explorer"){
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        detector_sub_ = this->create_subscription<leo_exploration::msg::FrontierClusters>("/frontier_centroids", 10, std::bind(&FrontierExplorer::detector_callback, this, _1));

        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("/odom",10, std::bind(&FrontierExplorer::odom_callback, this, _1));
        
        mode_sub_ = this->create_subscription<std_msgs::msg::Bool>("/auto_mode", rclcpp::QoS(10).transient_local(), std::bind(&FrontierExplorer::mode_callback, this, _1));

        explorer_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/goal_pose", 10);

    }

    private:
        rclcpp::Subscription<leo_exploration::msg::FrontierClusters>::SharedPtr detector_sub_; // Subscriber for detector
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_; // Subscriber for robot pose
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr mode_sub_; //Subscriber for auto_mode
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr explorer_pub_; // Publisher for nav2

        std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

        leo_exploration::msg::FrontierClusters::SharedPtr latest_centroids_;
        nav_msgs::msg::Odometry::SharedPtr latest_odom_;
        bool auto_mode_enabled_ = false;
        double robot_yaw_, robot_y_, robot_x_;

        std::vector<Frontier> frontier_list_;


        void detector_callback(const leo_exploration::msg::FrontierClusters::SharedPtr msg){
            latest_centroids_ = msg;
            frontier_list_.clear();

            // Check if we have odometry data
            if (!latest_odom_) {
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
                auto q = pose_map.pose.orientation; // need to convert to find yaw
                robot_yaw_ = atan2(2.0*(q.y*q.z + q.w*q.x), q.w*q.w - q.x*q.x - q.y*q.y + q.z*q.z);

            } catch (tf2::TransformException &ex) {
                RCLCPP_WARN(this->get_logger(), "Could not transform: %s", ex.what());
                return;
            }


            // loop throug and assign the data to the list and calcualte the metrics
            for (const auto& cluster : latest_centroids_->clusters){ 
                Frontier f;
                // Default data
                f.x = cluster.x;
                f.y = cluster.y;
                f.size = cluster.size;

                // Calculate distance
                double dx = f.x - robot_x_;
                double dy = f.y - robot_y_;
                f.distance = std::sqrt(dx*dx + dy*dy);

                // Calculate heading 
                double bearing = atan2(dy, dx);
                f.heading = bearing - robot_yaw_;


                f.score = calculate_score(f);

                frontier_list_.push_back(f);
            }

            // After populating frontier_list_
            if (frontier_list_.empty()) {
                RCLCPP_WARN(this->get_logger(), "No frontiers to explore");
                return;
            }

            // Find frontier with minimum score
            auto best = std::min_element(frontier_list_.begin(), frontier_list_.end(),
                [](const Frontier& a, const Frontier& b) {
                    return a.score < b.score;
                });

            // Publish the best frontier
            publish_goal(*best);


        }

        void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg){
            latest_odom_ = msg;
        }

        void mode_callback (const std_msgs::msg::Bool::SharedPtr msg){
            auto_mode_enabled_ = msg->data;
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
            RCLCPP_INFO(this->get_logger(), "Published goal: (%.2f, %.2f) with score %.2f", 
                        frontier.x, frontier.y, frontier.score);
        }

        double calculate_score(const Frontier& f){
            // Weights
            double w_distance = 1;
            double w_heading = 0;
            double w_size = 0;

            double score = w_distance*f.distance + w_heading*std::abs(f.heading) + w_size*f.size;

            return score;
        }

};

int main(int argc, char** argv){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FrontierExplorer>());
    rclcpp::shutdown();
    return 0;
}