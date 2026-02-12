#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/bool.hpp"

class ModeNode : public rclcpp::Node{
public:
    ModeNode() : Node("mode_node"), state_(0)
    {
        // Subscriber to /joy
        joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "/joy", 10,
            std::bind(&ModeNode::joy_callback, this, std::placeholders::_1));
        
        // Publisher to /auto_mode
        mode_pub_ = this->create_publisher<std_msgs::msg::Bool>("/auto_mode", rclcpp::QoS(10).transient_local());
        
        // Timer at 2Hz (every 0.5 seconds)
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(250),
            std::bind(&ModeNode::timer_callback, this));
        
        // Publish initial state immediately
        auto mode_msg = std_msgs::msg::Bool();
        mode_msg.data = false;
        mode_pub_->publish(mode_msg);
        
        RCLCPP_INFO(this->get_logger(), "Mode node started - initial state: DISABLED");
    }

private:
    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
    {
        latest_joy_ = msg;
    }
    
    void timer_callback()
    {
        if (!latest_joy_) {
            return;  // No joy message received yet
        }
        
        // Extract button states
        bool button_0 = latest_joy_->buttons[0];
        bool button_1 = latest_joy_->buttons[1];
        bool button_7 = latest_joy_->buttons[7];
        
        auto mode_msg = std_msgs::msg::Bool();
        
        if (state_ == 0) {
            // Check if RB + A pressed
            if (button_7 && button_0) {
                state_ = 1;
                mode_msg.data = true;
                mode_pub_->publish(mode_msg);
                //RCLCPP_INFO(this->get_logger(), "Autonomous mode ENABLED");
            }
        }
        else if (state_ == 1) {
            // Check if RB + B pressed
            if (button_7 && button_1) {
                state_ = 0;
                mode_msg.data = false;
                mode_pub_->publish(mode_msg);
                //RCLCPP_INFO(this->get_logger(), "Autonomous mode DISABLED");
            }
        }
    }
    
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr mode_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    sensor_msgs::msg::Joy::SharedPtr latest_joy_;
    int state_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ModeNode>());
    rclcpp::shutdown();
    return 0;
}