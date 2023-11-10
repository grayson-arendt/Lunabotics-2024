#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

class InitialPosePublisher : public rclcpp::Node
{
public:
    InitialPosePublisher() : Node("initial_pose_publisher")
    {
        pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("initialpose", 10);
    }
    void initial_pose()
    {
        auto pose = geometry_msgs::msg::PoseStamped();
        pose.header.frame_id = "odom";
        pose.pose.position.x = 0.0;
        pose.pose.position.y = 0.0;
        pose_publisher_->publish(pose);
    }
private:
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher_;
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