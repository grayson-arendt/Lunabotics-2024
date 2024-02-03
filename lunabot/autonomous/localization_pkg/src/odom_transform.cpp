#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"

/*
Author: Grayson Arendt

This node subscribes to the wheel odometry and creates the odom->base_link transform.
*/

class OdomTransform : public rclcpp::Node {
public:
    OdomTransform() : Node("odom_transform") {

        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "filtered_odom", 10,
            std::bind(&OdomTransform::transform_odom, this, std::placeholders::_1));
    }

private:

    void transform_odom(const nav_msgs::msg::Odometry::SharedPtr odom) {
        geometry_msgs::msg::TransformStamped odom_transform;

        odom_transform.header.stamp = this->get_clock()->now();
        odom_transform.header.frame_id = "odom";
        odom_transform.child_frame_id = "base_link";
        odom_transform.transform.translation.x = odom->pose.pose.position.x;
        odom_transform.transform.translation.y = odom->pose.pose.position.y;
        odom_transform.transform.translation.z = 0.0;
        odom_transform.transform.rotation.x = odom->pose.pose.orientation.x;
        odom_transform.transform.rotation.y = odom->pose.pose.orientation.y;
        odom_transform.transform.rotation.z = odom->pose.pose.orientation.z;
        odom_transform.transform.rotation.w = odom->pose.pose.orientation.w;

        tf_broadcaster_->sendTransform(odom_transform);
    }

    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;
};

int main(int argc, char **argv) {

    rclcpp::init(argc, argv);
    auto node = std::make_shared<OdomTransform>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
