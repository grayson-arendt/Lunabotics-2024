#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
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
            "t265/pose/sample", 10, std::bind(&OdometryTransform::transform_odom, this, std::placeholders::_1));
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

        tf2::Quaternion quaternion(odometry->pose.pose.orientation.x, odometry->pose.pose.orientation.y,
                                   odometry->pose.pose.orientation.z, odometry->pose.pose.orientation.w);
        tf2::Matrix3x3 euler(quaternion);

        euler.getRPY(roll, pitch, yaw);
        quaternion.setRPY(0.0, 0.0, yaw);

        odometry_transform.transform.rotation.x = quaternion.x();
        odometry_transform.transform.rotation.y = quaternion.y();
        odometry_transform.transform.rotation.z = quaternion.z();
        odometry_transform.transform.rotation.w = quaternion.w();

        tf_broadcaster_->sendTransform(odometry_transform);
    }

    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscriber_;
    double roll, pitch, yaw;
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