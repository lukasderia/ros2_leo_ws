#include <memory>
#include <chrono>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "leo_exploration/msg/frontier_clusters.hpp"

using std::placeholders::_1;

class FrontierExplorer : public rclcpp::Node{
    public:
    FrontierExplorer() : Node("frontier_explorer"){
        detector_sub_ = this->create_subscription<leo_exploration::msg::FrontierClusters>("/frontier_clentroids", 10, std::bind(&FrontierExplorer::detector_callback, this, _1));

        explorer_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/goal_pose", 10);

    }

    private:
        rclcpp::Subscription<leo_exploration::msg::FrontierClusters>::SharedPtr detector_sub_;
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr explorer_pub_;
        leo_exploration::msg::FrontierClusters::SharedPtr latest_centroids_;


        void detector_callback(const leo_exploration::msg::FrontierClusters::SharedPtr msg){
            latest_centroids_ = msg;

            //calculate som stuff then narrow down on one centroid to be published
            publish_goal();

        }

        void publish_goal(){
            // Method for pblishing the best goal pose based on some stats
        }

};

int main(int argc, char** argv){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FrontierExplorer>());
    rclcpp::shutdown();
    return 0;
}