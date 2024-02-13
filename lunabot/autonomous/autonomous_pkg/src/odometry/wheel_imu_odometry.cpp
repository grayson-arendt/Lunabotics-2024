#include "autonomous_msgs/msg/encoder.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include <chrono>
#include <cmath>
#include <thread>

/**
 * @brief Combines encoder and IMU data to calculate odometry.
 *
 * @author Grayson Arendt
 */
class WheelIMUOdometry : public rclcpp::Node
{

  public:
    /**
     * @brief Constructor for WheelIMUOdometry.
     */
    WheelIMUOdometry() : Node("wheel_imu_odometry")
    {

        wheel_joints_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);

        odometry_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("wheel_odometry", 10);

        encoder_subscriber_ = this->create_subscription<autonomous_msgs::msg::Encoder>(
            "encoders", 10, std::bind(&WheelIMUOdometry::publish_odom_tf_joints, this, std::placeholders::_1));

        imu_subscriber_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "imu/data", 10, std::bind(&WheelIMUOdometry::imu_callback, this, std::placeholders::_1));
    }

  private:
    /**
     * @brief Callback function for IMU data.
     *
     * @param imu_values The received IMU message.
     */
    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr imu_values)
    {

        raw_x = imu_values->orientation.x;
        raw_y = imu_values->orientation.y;
        raw_z = imu_values->orientation.z;
        raw_w = imu_values->orientation.w;

        tf2::Quaternion raw_quaternion(raw_x, raw_y, raw_z, raw_w);

        raw_quaternion.normalize();
        current_quaternion = raw_quaternion;

        tf2::Matrix3x3 current_euler(current_quaternion);
        current_euler.getRPY(current_roll, current_pitch, current_yaw);

        // RCLCPP_INFO(this->get_logger(), "Current roll: %.2f Current pitch: %.2f Current yaw: %.2f", current_roll,
        // current_pitch, current_yaw);
    }

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

        x_pose += distance_traveled * cos(current_yaw);
        y_pose += distance_traveled * sin(current_yaw);

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

    double r_encoder_final, l_encoder_final, r_encoder_initial, l_encoder_initial;
    double left_wheel_distance, right_wheel_distance;
    double current_roll, current_pitch, current_yaw;
    double raw_x, raw_y, raw_z, raw_w;
    double imu_x, imu_y, imu_z, imu_w;
    double right_wheel_pose_rad, left_wheel_pose_rad;
    double left_rad, right_rad;
    double distance_traveled;
    double x_pose = 0;
    double y_pose = 0;
    double theta_pose = 0;
    const double encoder_ticks = 91500;
    const double pi = 3.141592653589793;

    tf2::Quaternion current_quaternion;

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr wheel_joints_;
    rclcpp::Subscription<autonomous_msgs::msg::Encoder>::SharedPtr encoder_subscriber_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscriber_;
};

/**
 * @brief Main function.
 *
 * Initializes and spins the WheelIMUOdometry node.
 */
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WheelIMUOdometry>());
    rclcpp::shutdown();
    return 0;
}
