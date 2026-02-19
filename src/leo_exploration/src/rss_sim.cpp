#include "rclcpp/rclcpp.hpp"
#include <random>
#include <chrono>
#include <tuple>
#include <vector>
#include <Eigen/Dense>
#include <algorithm>
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "visualization_msgs/msg/marker_array.hpp"
#include <visualization_msgs/msg/marker.hpp>
#include "geometry_msgs/msg/vector3_stamped.hpp"

struct RSSMeas {
    double x, y, rss;
};

class RSSNodeSim : public rclcpp::Node{
    public:
        RSSNodeSim() : Node("RSSNodeSim"), generator_(std::chrono::system_clock::now().time_since_epoch().count()), distribution_(0.0, 3.0){

            this->declare_parameter("router_x", -9.0);
            this->declare_parameter("router_y", 9.0);
            router_x_ = this->get_parameter("router_x").as_double();
            router_y_ = this->get_parameter("router_y").as_double();
            tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
            tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

            odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("/odom",10, std::bind(&RSSNodeSim::odomCallback, this, std::placeholders::_1));

            rss_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/rss", 10);

            gradient_viz_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("gradient_arrow", 10);
            gradient_pub_ = this->create_publisher<geometry_msgs::msg::Vector3Stamped>("/rss_gradient", 10);
            // Timer for periodic scanning 
            scan_timer_ = this->create_wall_timer(std::chrono::milliseconds(500),std::bind(&RSSNodeSim::scanCallback, this));      
        }

    private:

        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr rss_pub_;
        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr gradient_viz_pub_;
        rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr gradient_pub_;

        std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
        rclcpp::TimerBase::SharedPtr scan_timer_;  // Add this line

        std::vector<RSSMeas> rss_buffer_;  // Add this

        double current_x_ = 0.0;
        double current_y_ = 0.0;
        
        double router_x_;
        double router_y_;

        std::mt19937 generator_;
        std::normal_distribution<double> distribution_;

        void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg){
            // Transform odom msg from odom frame to map frame
            try {
                // Get transform from odom to map
                geometry_msgs::msg::TransformStamped transform_stamped = 
                    tf_buffer_->lookupTransform("map", "odom", tf2::TimePointZero);

                // Transform robot pose from odom to map
                geometry_msgs::msg::PoseStamped pose_odom, pose_map;
                pose_odom.header = msg->header;
                pose_odom.pose = msg->pose.pose;

                tf2::doTransform(pose_odom, pose_map, transform_stamped);

                // Extract robot pose in map frame
                current_x_ = pose_map.pose.position.x;
                current_y_ = pose_map.pose.position.y;

            } catch (tf2::TransformException &ex) {
                RCLCPP_WARN(this->get_logger(), "Could not transform odom to map: %s", ex.what());
                return;
            }
        }

        void scanCallback(){
            double rss = getRSSMeasurement();
            
            // Add to buffer
            rss_buffer_.push_back({current_x_, current_y_, rss});
            
            // Publish accumulated cloud
            publishRSS();
            
            // Calculate and publish gradient (if enough points)
            auto [grad_x, grad_y] = calculateRSSGradient();
            if (grad_x != 0.0 || grad_y != 0.0) {
                RCLCPP_INFO(this->get_logger(), "RSS Gradient: [%.3f, %.3f]", grad_x, grad_y);
            }
        }

        double getRSSMeasurement(){
            // Simple model for signal stregnth decay
            double dx = current_x_- router_x_;
            double dy = current_y_- router_y_;
            
            double distance = std::sqrt(dx*dx + dy*dy);

            double noise = distribution_(generator_);

            return -30.0 - 20.0*log10(distance) + noise;
        }

        std::pair<double, double> calculateRSSGradient(){
            const double RADIUS = 4.0;
            const int MIN_POINTS = 100;
            
            // Safety check: need minimum measurements
            if (rss_buffer_.size() < 10) {
                RCLCPP_WARN(this->get_logger(), "Few points (%zu), waiting for more...", 
                            rss_buffer_.size());
                return {0.0, 0.0};  // No gradient available
            }

            // 1. Create lsit of all points their distance to robot
            std::vector<std::pair<double, RSSMeas>> dist_meas_pairs;
            for (const auto& meas : rss_buffer_){
                double dx = meas.x - current_x_;
                double dy = meas.y - current_y_;
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
            publishGradient(a,b);
            
            // 4. Return gradient
            return {a, b};
        }

        void publishRSS(){
            if (rss_buffer_.empty()) return;
            
            sensor_msgs::msg::PointCloud2 cloud;
            cloud.header.stamp = this->now();
            cloud.header.frame_id = "map";
            
            // Publish ALL accumulated points
            cloud.height = 1;
            cloud.width = rss_buffer_.size();
            cloud.is_bigendian = false;
            cloud.is_dense = true;
            
            // Define 4 fields: x, y, z, intensity
            cloud.fields.resize(4);
            cloud.fields[0].name = "x";
            cloud.fields[0].offset = 0;
            cloud.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
            cloud.fields[0].count = 1;
            
            cloud.fields[1].name = "y";
            cloud.fields[1].offset = 4;
            cloud.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
            cloud.fields[1].count = 1;
            
            cloud.fields[2].name = "z";
            cloud.fields[2].offset = 8;
            cloud.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
            cloud.fields[2].count = 1;
            
            cloud.fields[3].name = "intensity";
            cloud.fields[3].offset = 12;
            cloud.fields[3].datatype = sensor_msgs::msg::PointField::FLOAT32;
            cloud.fields[3].count = 1;
            
            cloud.point_step = 16;  // 4 fields * 4 bytes
            cloud.row_step = 16 * rss_buffer_.size();
            
            // Allocate data for all points
            cloud.data.resize(16 * rss_buffer_.size());
            
            // Copy all points into data buffer
            for (size_t i = 0; i < rss_buffer_.size(); i++) {
                float x_val = static_cast<float>(rss_buffer_[i].x);
                float y_val = static_cast<float>(rss_buffer_[i].y);
                float z_val = 0.0f;
                float rss_val = static_cast<float>(rss_buffer_[i].rss);
                
                size_t offset = i * 16;
                memcpy(&cloud.data[offset + 0], &x_val, sizeof(float));
                memcpy(&cloud.data[offset + 4], &y_val, sizeof(float));
                memcpy(&cloud.data[offset + 8], &z_val, sizeof(float));
                memcpy(&cloud.data[offset + 12], &rss_val, sizeof(float));
            }
            
            rss_pub_->publish(cloud);
        }

        void publishGradientVisualization(double grad_x, double grad_y){
            // Normalize to unit length
            double magnitude = std::sqrt(grad_x * grad_x + grad_y * grad_y);
            if (magnitude > 0) {
                grad_x /= magnitude;
                grad_y /= magnitude;
            } else {
                return;  // No direction to visualize
            }

            visualization_msgs::msg::Marker arrow;
            arrow.id = 0;
            arrow.header.frame_id = "map";
            arrow.header.stamp = this->now();
            arrow.type = visualization_msgs::msg::Marker::ARROW;
            arrow.action = visualization_msgs::msg::Marker::ADD;

            geometry_msgs::msg::Point start, end;
            start.x = current_x_;
            start.y = current_y_;
            start.z = 0.5;

            double scale = 1.0;
            end.x = current_x_ + grad_x * scale;
            end.y = current_y_ + grad_y * scale;
            end.z = 0.5;

            arrow.points.push_back(start);
            arrow.points.push_back(end);

            arrow.scale.x = 0.1;
            arrow.scale.y = 0.2;
            arrow.color.r = 1.0;
            arrow.color.g = 0.0;
            arrow.color.b = 0.0;
            arrow.color.a = 1.0;

            gradient_viz_pub_->publish(arrow);
        }

        void publishGradient(double grad_x, double grad_y){
            geometry_msgs::msg::Vector3Stamped gradient_msg;
            gradient_msg.header.stamp = this->now();
            gradient_msg.header.frame_id = "map";
            gradient_msg.vector.x = grad_x;
            gradient_msg.vector.y = grad_y;
            gradient_msg.vector.z = 0.0;
            
            gradient_pub_->publish(gradient_msg);
        }

};

int main(int argc, char** argv){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RSSNodeSim>());
    rclcpp::shutdown();
    return 0;
}