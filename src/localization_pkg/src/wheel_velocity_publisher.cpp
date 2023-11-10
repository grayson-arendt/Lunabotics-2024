#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"

#define Phoenix_No_WPI
#include "ctre/Phoenix.h"
#include "ctre/phoenix/platform/Platform.hpp"
#include "ctre/phoenix/unmanaged/Unmanaged.h"
#include "ctre/phoenix/cci/Unmanaged_CCI.h"

using namespace ctre::phoenix;
using namespace ctre::phoenix::platform;
using namespace ctre::phoenix::motorcontrol;
using namespace ctre::phoenix::motorcontrol::can;

TalonFX left_wheel_motor(2);
TalonFX right_wheel_motor(3);


class VelocityPublisher : public rclcpp::Node
{
public:
    VelocityPublisher() : Node("velocity_publisher")
    {
        right_wheel_motor.SetInverted(true);

        right_velocity_publisher_ = this->create_publisher<std_msgs::msg::Int32>("right_velocity", 100);
        left_velocity_publisher_ = this->create_publisher<std_msgs::msg::Int32>("left_velocity", 100);
        timer_ = this->create_wall_timer(std::chrono::microseconds(100),
                                         std::bind(&VelocityPublisher::right_velocity_callback, this));
        timer_ = this->create_wall_timer(std::chrono::microseconds(100), 
                                         std::bind(&VelocityPublisher::left_velocity_callback,this));                            
    }
    void right_velocity_callback()
    {
        auto right_velocity = std_msgs::msg::Int32();
        right_velocity.data = right_wheel_motor.GetSelectedSensorVelocity();
        right_velocity_publisher_->publish(right_velocity);
    }
    void left_velocity_callback()
    {
        auto left_velocity = std_msgs::msg::Int32();
        left_velocity.data = left_wheel_motor.GetSelectedSensorVelocity();
        left_velocity_publisher_->publish(left_velocity);
    }

private:
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr right_velocity_publisher_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr left_velocity_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<VelocityPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}