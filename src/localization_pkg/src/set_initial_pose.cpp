#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"

class InitialPosePublisher : public rclcpp::Node
{
public:
    InitialPosePublisher() : Node("initial_pose_publisher")
    {
        pose_publisher = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("initialpose", 10);
    }
    void initial_pose()
    {
        auto pose = geometry_msgs::msg::PoseWithCovarianceStamped();
        pose.header.frame_id = "odom";
        pose.pose.pose.position.x = 0.0;
        pose.pose.pose.position.y = 0.0;
        pose_publisher->publish(pose);
    }
private:
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_publisher;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<InitialPosePublisher>();
    node->initial_pose();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}