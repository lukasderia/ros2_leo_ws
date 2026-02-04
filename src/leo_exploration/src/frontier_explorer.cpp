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

using std::placeholders::_1;

struct Frontier {
    double x;
    double y;
    int size;
    double distance;
    double heading;
    double score;
};

struct RSSMeas {
    double x, y, rss;
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

        rss_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>("/rss", 10, std::bind(&FrontierExplorer::rss_callback, this, _1));

        explorer_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/goal_pose", 10);

        gradient_viz_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("gradient_arrow", 10);

    }

    private:
        rclcpp::Subscription<leo_exploration::msg::FrontierClusters>::SharedPtr detector_sub_; // Subscriber for detector
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_; // Subscriber for robot pose
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr mode_sub_; //Subscriber for auto_mode
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr rss_sub_; // Subscriber for rss node
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr explorer_pub_; // Publisher for nav2
        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr gradient_viz_pub_;

        std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

        leo_exploration::msg::FrontierClusters::SharedPtr latest_centroids_;
        nav_msgs::msg::Odometry::SharedPtr latest_odom_;
        bool auto_mode_enabled_ = false;
        double robot_yaw_, robot_y_, robot_x_;
        bool odom_recieved_ = false;

        std::vector<Frontier> frontier_list_;
        std::vector<RSSMeas> rss_buffer_;

        // In private section
        bool has_published_goal_ = false;
        double last_goal_x_ = 0.0;
        double last_goal_y_ = 0.0;


        void detector_callback(const leo_exploration::msg::FrontierClusters::SharedPtr msg){
            latest_centroids_ = msg;
            frontier_list_.clear();

            // Check if we have odometry data
            if (!odom_recieved_) {  // Check the flag instead
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

            // Now we have valid robot pose, calculate gradient
            auto [grad_x, grad_y] = calculateRSSGradient();
            RCLCPP_INFO(this->get_logger(), "RSS Gradient: [%.3f, %.3f]", grad_x, grad_y);

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

            // Check if new goal is significantly different from last goal
            if (has_published_goal_) {
                double dx = best->x - last_goal_x_;
                double dy = best->y - last_goal_y_;
                double distance_to_last_goal = std::sqrt(dx*dx + dy*dy);
                
                double threshold = 0.50;  // meters - adjust as needed
                
                if (distance_to_last_goal < threshold) {
                    RCLCPP_DEBUG(this->get_logger(), "New goal too close to last goal (%.2f m), skipping", distance_to_last_goal);
                    return;  // Skip publishing
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

        void rss_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
            // Should only be 1 point per message
            float x, y, z, rss;
            memcpy(&x, &msg->data[0], sizeof(float));
            memcpy(&y, &msg->data[4], sizeof(float));
            memcpy(&z, &msg->data[8], sizeof(float));
            memcpy(&rss, &msg->data[12], sizeof(float));
            
            rss_buffer_.push_back({static_cast<double>(x), 
                                static_cast<double>(y), 
                                static_cast<double>(rss)});
                                
            RCLCPP_DEBUG(this->get_logger(), "RSS buffer size: %zu", rss_buffer_.size());
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

        double calculate_score(const Frontier& f){
            // Weights
            double w_distance = 1;
            double w_heading = 0;
            double w_size = 0;

            double score = w_distance*f.distance + w_heading*std::abs(f.heading) + w_size*f.size;

            return score;
        }

        std::pair<double, double> calculateRSSGradient() {
            const double RADIUS = 2.0;
            const int MIN_POINTS = 10;
            
            // Safety check: need minimum measurements
            if (rss_buffer_.size() < MIN_POINTS) {
                RCLCPP_WARN(this->get_logger(), "Not enough RSS measurements (%zu) for gradient calculation", 
                            rss_buffer_.size());
                return {0.0, 0.0};  // No gradient available
            }

            // 1. Create lsit of all points their distance to robot
            std::vector<std::pair<double, RSSMeas>> dist_meas_pairs;
            for (const auto& meas : rss_buffer_){
                double dx = meas.x - robot_x_;
                double dy = meas.y - robot_y_;
                double dist = std::sqrt(dx*dx + dy*dy);

                dist_meas_pairs.push_back({dist, meas});
            }

            // 2. Sort list based on lowest distance
            std::sort(dist_meas_pairs.begin(), dist_meas_pairs.end(), [](const auto& a, const auto& b){
                return a.first < b.first;
            });

            
            // 3.1 Find points within radius
            std::vector<RSSMeas> nearby_points;
            for (const auto& [dist, meas] : dist_meas_pairs) {                
                if (dist <= RADIUS) {
                    nearby_points.push_back(meas);
                } else {
                    break;
                }
            }

            // 3.2 If not enough, fill up with closest point
            while (nearby_points.size() < MIN_POINTS && nearby_points.size() < dist_meas_pairs.size()) {
                nearby_points.push_back(dist_meas_pairs[nearby_points.size()].second);
            }
            
            // 4. Fit plane using least squares
            size_t n = nearby_points.size();

            Eigen::MatrixXd A(n, 3);
            Eigen::VectorXd z(n);
            for (size_t i = 0; i < n; i++){
                A(i, 0) = nearby_points[i].x;
                A(i, 1) = nearby_points[i].y;
                A(i, 2) = 1.0;
                z(i) = nearby_points[i].rss;
            }

            // Solve equation Ax=z where x holds the a,b and c coefficients of the plane
            Eigen::Vector3d x = A.colPivHouseholderQr().solve(z);
            double a = x(0);
            double b = x(1);

            publishGradientVisualization(a, b);
            
            // 4. Return gradient
            return {a, b};

        }

        void publishGradientVisualization(double grad_x, double grad_y) {
            visualization_msgs::msg::Marker arrow;
            arrow.id = 0;
            arrow.header.frame_id = "map";
            arrow.header.stamp = this->now();
            arrow.type = visualization_msgs::msg::Marker::ARROW;
            arrow.action = visualization_msgs::msg::Marker::ADD;

            // Arrow starts at robot position
            geometry_msgs::msg::Point start, end;
            start.x = robot_x_;
            start.y = robot_y_;
            start.z = 0.5;  // Raise it up so it's visible

            // Arrow points in gradient direction (scale for visibility)
            double scale = 2.0;  // Adjust arrow length
            end.x = robot_x_ + grad_x * scale;
            end.y = robot_y_ + grad_y * scale;
            end.z = 0.5;

            arrow.points.push_back(start);
            arrow.points.push_back(end);

            arrow.scale.x = 0.1;  // Shaft diameter
            arrow.scale.y = 0.2;  // Head diameter
            arrow.color.r = 1.0;
            arrow.color.g = 0.0;
            arrow.color.b = 0.0;
            arrow.color.a = 1.0;

            gradient_viz_pub_->publish(arrow);
        }     
};

int main(int argc, char** argv){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FrontierExplorer>());
    rclcpp::shutdown();
    return 0;
}