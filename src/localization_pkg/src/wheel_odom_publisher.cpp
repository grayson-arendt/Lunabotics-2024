#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <math.h>
#include <chrono>
#include <thread>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "sensor_msgs/msg/joint_state.hpp"

#define Phoenix_No_WPI
#include "ctre/Phoenix.h"
#include "ctre/phoenix/platform/Platform.hpp"
#include "ctre/phoenix/unmanaged/Unmanaged.h"
#include "ctre/phoenix/cci/Unmanaged_CCI.h"

using namespace ctre::phoenix;
using namespace ctre::phoenix::platform;
using namespace ctre::phoenix::motorcontrol;
using namespace ctre::phoenix::motorcontrol::can;

std::string interface = "can0";
TalonFX left_wheel_motor(2, interface);
TalonFX right_wheel_motor(3);
/* look into getting this working, right now it gives an error
TalonFXConfiguration config;
config.supplyCurrLimit.enable = true;
config.supplyCurrLimit.triggerThresholdCurrent = 40;
left_wheel_motor.ConfigAllSettings(config);
right_wheel_motor.ConfigAllSettings(config);
*/

class OdomPublisher : public rclcpp::Node
{
public:
    OdomPublisher()
        : Node("wheel_odom_pub")
    {
        right_wheel_motor.SetInverted(true);

        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        wheel_joints_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);

        odom_data_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("wheel_odom", 10);
        
        timer_ = this->create_wall_timer(
            std::chrono::microseconds(10),
            std::bind(&OdomPublisher::publish_odom_tf_joints, this));
    }

private:
    void publish_odom_tf_joints()
    {
        ctre::phoenix::unmanaged::Unmanaged::FeedEnable(10000);
        r_encoder_initial = right_wheel_motor.GetSelectedSensorPosition();
        l_encoder_initial = left_wheel_motor.GetSelectedSensorPosition();
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        r_encoder_final = right_wheel_motor.GetSelectedSensorPosition();
        l_encoder_final = left_wheel_motor.GetSelectedSensorPosition();

        right_rad = (((r_encoder_final - r_encoder_initial) * 2.0 * pi) / encoder_ticks);
        left_rad = (((l_encoder_final - l_encoder_initial) * 2.0 * pi) / encoder_ticks);

        right_wheel_distance = right_rad * 0.1016; 
        left_wheel_distance = left_rad * 0.1016;

        distance_traveled = (right_wheel_distance + left_wheel_distance) / 2.0;
        delta_theta = (right_wheel_distance - left_wheel_distance) / (2.0 * wheel_to_base_footprint);

        x_pose += distance_traveled * cos(theta_pose + (delta_theta / 2));
        y_pose += distance_traveled * sin(theta_pose + (delta_theta / 2));
        theta_pose += delta_theta;

        auto wheel_odom = nav_msgs::msg::Odometry();
        wheel_odom.header.stamp = this->get_clock()->now();
        wheel_odom.header.frame_id = "odom";
        wheel_odom.child_frame_id = "base_footprint";
        wheel_odom.pose.pose.position.x = x_pose;
        wheel_odom.pose.pose.position.y = y_pose;
        wheel_odom.pose.pose.orientation.z = theta_pose;
        wheel_odom.pose.pose.orientation.w = 1.0;

        geometry_msgs::msg::TransformStamped odom_basefootprint_tf;
        odom_basefootprint_tf.header.stamp = this->get_clock()->now();
        odom_basefootprint_tf.header.frame_id = "odom";
        odom_basefootprint_tf.child_frame_id = "base_footprint";
        odom_basefootprint_tf.transform.translation.x = x_pose;
        odom_basefootprint_tf.transform.translation.y = y_pose;
        odom_basefootprint_tf.transform.translation.z = 0.0;
        odom_basefootprint_tf.transform.rotation.x = 0.0;
        odom_basefootprint_tf.transform.rotation.y = 0.0;
        odom_basefootprint_tf.transform.rotation.z = theta_pose;
        odom_basefootprint_tf.transform.rotation.w = 1.0;

        right_wheel_pose_rad += right_rad;
        left_wheel_pose_rad += left_rad;

        auto wheel_joints = sensor_msgs::msg::JointState();
        wheel_joints.header.stamp = this->get_clock()->now();
        wheel_joints.name = {"base_rear_right_wheel_joint", "base_rear_left_wheel_joint"};
        wheel_joints.position = {right_wheel_pose_rad, left_wheel_pose_rad};


        tf_broadcaster_->sendTransform(odom_basefootprint_tf);
        wheel_joints_->publish(wheel_joints);
        odom_data_pub_->publish(wheel_odom);
    }

    double r_encoder_initial;
    double r_encoder_final;
    double l_encoder_initial;
    double l_encoder_final;
    double left_wheel_distance;
    double right_wheel_distance;
    const double wheel_to_base_footprint = 8.90625;
    const double encoder_ticks = 70000; //I measured about 91500 ticks per revolution using the GetSelectedSensorPosition function
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
    rclcpp::TimerBase::SharedPtr timer_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr wheel_joints_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    right_wheel_motor.SetSelectedSensorPosition(0.0);
    left_wheel_motor.SetSelectedSensorPosition(0.0);
    auto node = std::make_shared<OdomPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
