#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"

/**
 * @brief Broadcasts the odom->base_link transform.
 *
 * @author Grayson Arendt
 */
class OdometryTransform : public rclcpp::Node
{

public:
    /**
     * @brief Constructor for OdometryTransform.
     */
    OdometryTransform() : Node("odometry_transform")
    {
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        odometry_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "odometry/filtered", 10,
            std::bind(&OdometryTransform::transform_odom, this, std::placeholders::_1));
    }

private:
    /**
     * @brief Callback function to transform odometry and broadcast the odometry to base_link transform.
     *
     * @param odometry The received odometry message.
     */
    void transform_odom(const nav_msgs::msg::Odometry::SharedPtr odometry)
    {
        geometry_msgs::msg::TransformStamped odometry_transform;

        odometry_transform.header.stamp = this->get_clock()->now();
        odometry_transform.header.frame_id = "odom";
        odometry_transform.child_frame_id = "base_link";
        odometry_transform.transform.translation.x = odometry->pose.pose.position.x;
        odometry_transform.transform.translation.y = odometry->pose.pose.position.y;
        odometry_transform.transform.translation.z = 0.0;
        odometry_transform.transform.rotation.x = odometry->pose.pose.orientation.x;
        odometry_transform.transform.rotation.y = odometry->pose.pose.orientation.y;
        odometry_transform.transform.rotation.z = odometry->pose.pose.orientation.z;
        odometry_transform.transform.rotation.w = odometry->pose.pose.orientation.w;

        tf_broadcaster_->sendTransform(odometry_transform);
    }

    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscriber_;
};

/**
 * @brief Main function.
 *
 * Initializes and spins the OdometryTransform node.
 */
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<OdometryTransform>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
