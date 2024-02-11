#include <math.h>
#include <cmath>
#include <chrono>
#include <thread>
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "autonomous_msgs/msg/encoder.hpp"
#include "tf2/LinearMath/Quaternion.h"

/**
 * @brief Uses only encoder data to calculate odometry.
 *
 * @author Anthony Baran
 * @author Grayson Arendt
 */
class WheelOdometry : public rclcpp::Node
{

public:
    /**
     * @brief Constructor for WheelOdometry.
     */
    WheelOdometry() : Node("wheel_odometry")
    {

        wheel_joints_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);

        odometry_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("wheel_odometry", 10);

        encoder_subscriber_ = this->create_subscription<autonomous_msgs::msg::Encoder>(
            "encoders", 10,
            std::bind(&WheelOdometry::publish_odom_tf_joints, this, std::placeholders::_1));
    }

private:
    /**
     * @brief Callback function for encoder data and publishes odometry and joint states.
     *
     * @param encoder_values The received encoder message.
     */
    void publish_odom_tf_joints(const autonomous_msgs::msg::Encoder::SharedPtr encoder_values)
    {
        l_encoder_initial = encoder_values->left_initial;
        r_encoder_initial = encoder_values->right_initial;
        l_encoder_final = encoder_values->left_final;
        r_encoder_final = encoder_values->right_final;

        right_rad = (2.0 * pi * (r_encoder_final - r_encoder_initial)) / encoder_ticks;
        left_rad = (2.0 * pi * (l_encoder_final - l_encoder_initial)) / encoder_ticks;

        right_wheel_distance = right_rad * 0.1016;
        left_wheel_distance = left_rad * 0.1016;

        distance_traveled = (right_wheel_distance + left_wheel_distance) / 2.0;
        delta_theta = (right_wheel_distance - left_wheel_distance) / (2.0 * wheel_to_base_link);

        x_pose += distance_traveled * cos(theta_pose + delta_theta * 0.5);
        y_pose += distance_traveled * sin(theta_pose + delta_theta * 0.5);

        theta_pose += delta_theta;
        theta_pose = std::fmod(theta_pose + pi, 2 * pi) - pi;

        tf2::Quaternion current_quaternion;

        current_quaternion.setRPY(0, 0, theta_pose);
        current_quaternion.normalize();

        auto wheel_odometry = nav_msgs::msg::Odometry();
        wheel_odometry.header.stamp = this->get_clock()->now();

        wheel_odometry.header.frame_id = "odom";
        wheel_odometry.child_frame_id = "base_link";
        wheel_odometry.pose.pose.position.x = x_pose;
        wheel_odometry.pose.pose.position.y = y_pose;

        wheel_odometry.pose.pose.orientation.x = current_quaternion.x();
        wheel_odometry.pose.pose.orientation.y = current_quaternion.y();
        wheel_odometry.pose.pose.orientation.z = current_quaternion.z();
        wheel_odometry.pose.pose.orientation.w = current_quaternion.w();

        right_wheel_pose_rad += right_rad;
        left_wheel_pose_rad += left_rad;

        auto wheel_joints = sensor_msgs::msg::JointState();
        wheel_joints.header.stamp = this->get_clock()->now();
        wheel_joints.name = {"base_rear_right_wheel_joint", "base_rear_left_wheel_joint"};
        wheel_joints.position = {right_wheel_pose_rad, left_wheel_pose_rad};

        odometry_publisher_->publish(wheel_odometry);
        wheel_joints_->publish(wheel_joints);
    }

    double r_encoder_final;
    double l_encoder_final;
    double r_encoder_initial;
    double l_encoder_initial;
    double left_wheel_distance;
    double right_wheel_distance;
    const double wheel_to_base_link = 0.5207;
    const double encoder_ticks = 91500;
    const double pi = 3.141592653589793;
    double distance_traveled;
    double delta_theta;
    double x_pose = 0;
    double y_pose = 0;
    double theta_pose = 0;
    double right_wheel_pose_rad;
    double left_wheel_pose_rad;
    double left_rad;
    double right_rad;

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr wheel_joints_;
    rclcpp::Subscription<autonomous_msgs::msg::Encoder>::SharedPtr encoder_subscriber_;
};

/**
 * @brief Main function.
 *
 * Initializes and spins the WheelOdometry node.
 */
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<WheelOdometry>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
