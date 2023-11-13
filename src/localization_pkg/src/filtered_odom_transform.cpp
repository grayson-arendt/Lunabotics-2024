#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"


class FilteredOdomTF : public rclcpp::Node
{
public:
    FilteredOdomTF() : Node("filtered_odom_tf"){

        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        // This is getting the filtered odometry data from the ekf_filter_node from robot_localization package
        filtered_odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "wheel_odom", 10,
            std::bind(&FilteredOdomTF::transform_filtered_odom, this, std::placeholders::_1));
    }

private:

    void transform_filtered_odom(const nav_msgs::msg::Odometry::SharedPtr filtered_odom)
    {
        geometry_msgs::msg::TransformStamped filtered_odom_tf;
        filtered_odom_tf.header.stamp = this->get_clock()->now();
        filtered_odom_tf.header.frame_id = "odom";
        // changing all base_footprint to base_link
        filtered_odom_tf.child_frame_id = "base_link";
        filtered_odom_tf.transform.translation.x = filtered_odom->pose.pose.position.x;
        filtered_odom_tf.transform.translation.y = filtered_odom->pose.pose.position.y;
        filtered_odom_tf.transform.translation.z = 0.0;
        filtered_odom_tf.transform.rotation.x = filtered_odom->pose.pose.orientation.x;
        filtered_odom_tf.transform.rotation.y = filtered_odom->pose.pose.orientation.y;
        filtered_odom_tf.transform.rotation.z = filtered_odom->pose.pose.orientation.z;
        filtered_odom_tf.transform.rotation.w = filtered_odom->pose.pose.orientation.w;

        tf_broadcaster_->sendTransform(filtered_odom_tf);
    }

    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr filtered_odom_sub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<FilteredOdomTF>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
