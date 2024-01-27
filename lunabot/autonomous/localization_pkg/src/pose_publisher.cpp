#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"

/*
Author: Grayson Arendt

This node subscribes to the odometry topic and publishes a robot pose. 
*/

class PosePublisher : public rclcpp::Node {
public:
    PosePublisher() : Node("pose_pub"){

        pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("pose", 10);
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "wheel_odom", 10,
            std::bind(&PosePublisher::odom_callback, this, std::placeholders::_1));
    }

private:
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr odom) {

        auto pose_msg = geometry_msgs::msg::PoseWithCovarianceStamped();

        pose_msg.header.stamp = this->get_clock()->now();
        pose_msg.header.frame_id = "base_link";
        pose_msg.pose = odom->pose;

        pose_pub_->publish(pose_msg);
    }

    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PosePublisher>());
    rclcpp::shutdown();
    return 0;
}
