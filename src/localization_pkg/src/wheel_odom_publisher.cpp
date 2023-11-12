#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include <cmath>
#include <chrono>

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

using std::placeholders::_1;

class OdomPublisher : public rclcpp::Node
{
public:
    OdomPublisher()
        : Node("wheel_odom_pub")
    {
        // odom_data_pub = this->create_publisher<nav_msgs::msg::Odometry>("odom_data", 100);
        // timer_ = this->create_wall_timer(
        //     std::chrono::microseconds(10),
        //     std::bind(&OdomPublisher::publish_odom, this));
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        // timer1_ = this->create_wall_timer(
        //     std::chrono::milliseconds(10),
        //     std::bind(&OdomPublisher::broadcast_transform_callback, this));
        //  Initialize subscribers
        //  sub_for_right_velocity = this->create_subscription<std_msgs::msg::Int32>(
        //     "right_velocity", 100, std::bind(&OdomPublisher::publish_odom, this, _1));
        //  sub_for_left_velocity = this->create_subscription<std_msgs::msg::Int32>(
        //     "left_velocity", 100, std::bind(&OdomPublisher::publish_odom, this, _1));
    }
    void broadcast_transform_callback()
    {
        geometry_msgs::msg::TransformStamped t;

        t.header.stamp = this->get_clock()->now();
        t.header.frame_id = "odom";
        t.child_frame_id = "base_link";

        t.transform.translation.x = 1.0;
        t.transform.translation.y = 1.0;
        t.transform.translation.z = 0.0;
        t.transform.rotation.x = 0.0;
        t.transform.rotation.y = 0.0;
        t.transform.rotation.z = 0.0;
        t.transform.rotation.w = 1.0;

        tf_broadcaster_->sendTransform(t);
    }

private:
    void publish_odom()
    {
        auto o = nav_msgs::msg::Odometry();
        o.header.frame_id = "odom";
        o.child_frame_id = "base_link";
        o.pose.pose.position.x = 1.0;
        o.pose.pose.position.y = 1.0;
        o.pose.pose.position.z = 0.0;
        o.pose.pose.orientation.x = 0.0;
        o.pose.pose.orientation.y = 0.0;
        o.pose.pose.orientation.z = 0.0;
        o.pose.pose.orientation.w = 1.0;
        o.twist.twist.linear.x = 0.0;
        o.twist.twist.linear.y = 0.0;
        o.twist.twist.linear.z = 0.0;
        o.twist.twist.angular.x = 0.0;
        o.twist.twist.angular.y = 0.0;
        o.twist.twist.angular.z = 0.0;

        odom_data_pub->publish(o);
    }

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_data_pub;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr timer1_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    // rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sub_for_right_velocity;
    // rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sub_for_left_velocity;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<OdomPublisher>();
    node->broadcast_transform_callback();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
