#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <math.h>
#include <cmath>
#include <chrono>
#include <thread>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "autonomous_msgs/msg/encoder.hpp"

#include "tf2/LinearMath/Quaternion.h"
#include "sensor_msgs/msg/joint_state.hpp"

/*
Author: Anthony Baran
Modified by: Grayson Arendt

This node uses encoders only to calculate the position/orientation then publishes to wheel_odom.
*/

class OdomPublisher : public rclcpp::Node
{
public:
    OdomPublisher() : Node("wheel_odom_pub"){

        wheel_joints_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);

        odom_data_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("wheel_odom", 10);

        encoder_sub_ = this->create_subscription<autonomous_msgs::msg::Encoder>(
            "encoders", 10,
            std::bind(&OdomPublisher::publish_odom_tf_joints, this, std::placeholders::_1));
    }

private:

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
        delta_theta = (right_wheel_distance - left_wheel_distance) / (2.0 * wheel_to_base_footprint);

        x_pose += distance_traveled * cos(theta_pose + delta_theta*0.5);
        y_pose += distance_traveled * sin(theta_pose + delta_theta*0.5);
        theta_pose += delta_theta;

        // normalize the orientation angle to stay within [-pi, pi]
        theta_pose = std::fmod(theta_pose + pi, 2 * pi) - pi;

        tf2::Quaternion q;
        // roll, pitch, yaw - we're only using yaw since this is 2D
        q.setRPY(0, 0, theta_pose);
        q.normalize();

        auto wheel_odom = nav_msgs::msg::Odometry();
        wheel_odom.header.stamp = this->get_clock()->now();
        wheel_odom.header.frame_id = "odom";
        // changed to base_link
        wheel_odom.child_frame_id = "base_link";
        wheel_odom.pose.pose.position.x = x_pose;
        wheel_odom.pose.pose.position.y = y_pose;

        wheel_odom.pose.pose.orientation.x = q.x();
        wheel_odom.pose.pose.orientation.y = q.y();
        wheel_odom.pose.pose.orientation.z = q.z();
        wheel_odom.pose.pose.orientation.w = q.w();

        right_wheel_pose_rad += right_rad;
        left_wheel_pose_rad += left_rad;

        auto wheel_joints = sensor_msgs::msg::JointState();
        wheel_joints.header.stamp = this->get_clock()->now();
        wheel_joints.name = {"base_rear_right_wheel_joint", "base_rear_left_wheel_joint"};
        wheel_joints.position = {right_wheel_pose_rad, left_wheel_pose_rad};

        odom_data_pub_->publish(wheel_odom);
        wheel_joints_->publish(wheel_joints);
    }

    double r_encoder_final;
    double l_encoder_final;
    double r_encoder_initial;
    double l_encoder_initial;
    double left_wheel_distance;
    double right_wheel_distance;
    // base_footprint is underneath base_link
    const double wheel_to_base_footprint = 0.5207;
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
    
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_data_pub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr wheel_joints_;
    rclcpp::Subscription<autonomous_msgs::msg::Encoder>::SharedPtr encoder_sub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<OdomPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
