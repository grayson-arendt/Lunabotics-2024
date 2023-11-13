#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <math.h>
#include <cmath>
#include <chrono>
#include <thread>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "navigation_pkg/msg/encoder.hpp"

#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "sensor_msgs/msg/joint_state.hpp"
#include "sensor_msgs/msg/imu.hpp"

class OdomPublisher : public rclcpp::Node
{
public:
    OdomPublisher() : Node("wheel_odom_pub"){

        wheel_joints_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);

        odom_data_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("wheel_odom", 10);

        encoder_sub_ = this->create_subscription<navigation_pkg::msg::Encoder>(
            "encoders", 10,
            std::bind(&OdomPublisher::publish_odom_tf_joints, this, std::placeholders::_1));

        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "imu/data", 10,
            std::bind(&OdomPublisher::imu_callback, this, std::placeholders::_1));
        iteration = 0;

        yaw_values.push_back(0.0);
    }

private:

    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr imu_values) {

        // raw orientation values
        raw_x = imu_values->orientation.x;
        raw_y = imu_values->orientation.y;
        raw_z = imu_values->orientation.z;
        raw_w = imu_values->orientation.w;

        tf2::Quaternion raw_q(raw_x, raw_y, raw_z, raw_w);
        // make sure it's normalized
        raw_q.normalize();
        current_q = raw_q;
   
        tf2::Matrix3x3 m(current_q);
        m.getRPY(current_roll, current_pitch, current_yaw);

        // negative due to camera placement
        yaw_values.push_back(-current_yaw);

        //RCLCPP_INFO(this->get_logger(), "Current roll: %.2f Current pitch: %.2f Current yaw: %.2f", current_roll, current_pitch, current_yaw);

        iteration++;
    }

    void publish_odom_tf_joints(const navigation_pkg::msg::Encoder::SharedPtr encoder_values)
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
        
        delta_theta = yaw_values[iteration + 1] - yaw_values[iteration];
        delta_theta = std::atan2(std::sin(delta_theta), std::cos(delta_theta));

        theta_pose += delta_theta;
        x_pose += distance_traveled * cos(theta_pose + delta_theta*0.5);
        y_pose += distance_traveled * sin(theta_pose + delta_theta*0.5);
  
        auto wheel_odom = nav_msgs::msg::Odometry();
        wheel_odom.header.stamp = this->get_clock()->now();
        wheel_odom.header.frame_id = "odom";

        // changed to base_link
        wheel_odom.child_frame_id = "base_link";
        wheel_odom.pose.pose.position.x = x_pose;
        wheel_odom.pose.pose.position.y = y_pose;

        wheel_odom.pose.pose.orientation.x = current_q.x();
        wheel_odom.pose.pose.orientation.y = current_q.y();
        wheel_odom.pose.pose.orientation.z = current_q.z();
        wheel_odom.pose.pose.orientation.w = current_q.w();

        right_wheel_pose_rad += right_rad;
        left_wheel_pose_rad += left_rad;

        auto wheel_joints = sensor_msgs::msg::JointState();
        wheel_joints.header.stamp = this->get_clock()->now();
        wheel_joints.name = {"base_rear_right_wheel_joint", "base_rear_left_wheel_joint"};
        wheel_joints.position = {right_wheel_pose_rad, left_wheel_pose_rad};

        odom_data_pub_->publish(wheel_odom);
        wheel_joints_->publish(wheel_joints);
    }

    int iteration;
    double r_encoder_final, l_encoder_final, r_encoder_initial, l_encoder_initial;
    double left_wheel_distance, right_wheel_distance;
    double current_roll, current_pitch, current_yaw;
    double raw_x, raw_y, raw_z, raw_w;
    double normalized_x, normalized_y, normalized_z, normalized_w;
    double imu_x, imu_y, imu_z, imu_w;
    double right_wheel_pose_rad, left_wheel_pose_rad;
    double left_rad, right_rad;
    double distance_traveled;
    double delta_theta;
    double x_pose = 0;
    double y_pose = 0;
    double theta_pose = 0; 
    const double wheel_to_base_footprint = 0.5207;
    const double encoder_ticks = 91500; 
    const double pi = 3.141592653589793;

    std::vector<double> yaw_values;
    tf2::Quaternion current_q;
    
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_data_pub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr wheel_joints_;
    rclcpp::Subscription<navigation_pkg::msg::Encoder>::SharedPtr encoder_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<OdomPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
