#include "rclcpp/rclcpp.hpp"
#include <tuple>
#include <vector>
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

const std::string command = "iwconfig wlan1 | grep -oP 'Signal level=\\K-?\\d+'";

class RSSNode : public rclcpp::Node{
    public:
    RSSNode() : Node("RSSNode"){
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("/odometry_merged",10, std::bind(&RSSNode::odom_callback, this, std::placeholders::_1));

        rss_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/rss", 10);

        // Timer for periodic scanning (every 3 seconds)
        scan_timer_ = this->create_wall_timer(std::chrono::milliseconds(500),std::bind(&RSSNode::scanCallback, this));

    }

    private:

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr rss_pub_;
    rclcpp::TimerBase::SharedPtr scan_timer_;

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    
    double current_x_ = 0.0;
    double current_y_ = 0.0;

        void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg){
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
            double rss = getRSSMeasurement(command);

            publishRSS(current_x_, current_y_, rss);

        }
        double getRSSMeasurement(std::string command){
            FILE* pipe = popen(command.c_str(), "r");
            if (pipe == nullptr) {
                // Log error
                return 0.0;
            }

            char buffer[256];
            if (fgets(buffer, sizeof(buffer), pipe) != nullptr) {
                double rss_value = std::stod(buffer);
                pclose(pipe);  // Close before returning
                return rss_value;
            }
            pclose(pipe);  // Also close if fgets failed
            return 0.0;  // Return default if no data
        }

        void publishRSS(double x, double y, double rss){

            sensor_msgs::msg::PointCloud2 cloud;
            cloud.header.stamp = this->now();
            cloud.header.frame_id = "map";

            // Publish only single new point
            cloud.height = 1;
            cloud.width = 1;
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
            cloud.row_step = 16;    // point_step * 1 point
            
            // Allocate data for single point
            cloud.data.resize(16);
            
            // Copy new point into data buffer
            float x_val = static_cast<float>(x);
            float y_val = static_cast<float>(y);
            float z_val = 0.0f;
            float rss_val = static_cast<float>(rss);
            
            memcpy(&cloud.data[0], &x_val, sizeof(float));
            memcpy(&cloud.data[4], &y_val, sizeof(float));
            memcpy(&cloud.data[8], &z_val, sizeof(float));
            memcpy(&cloud.data[12], &rss_val, sizeof(float));

            rss_pub_->publish(cloud);
        }
};


int main(int argc, char** argv){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RSSNode>());
    rclcpp::shutdown();
    return 0;
}