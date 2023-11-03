#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "navigation_pkg/DifferentialDrive.hpp"

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

class MotorController : public rclcpp::Node
{
public:
    MotorController() : Node("motor_controller")
    {
        right_wheel_motor.SetInverted(true);

        motor_controller_subscriber = this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", 10,
            std::bind(&MotorController::callbackMotors, this, std::placeholders::_1));
    }

private:

    void callbackMotors(const geometry_msgs::msg::Twist::SharedPtr cmd_vel)
    {
        float x_linear = cmd_vel->linear.x;
        float z_angular = cmd_vel->angular.z;

        DifferentialDrive d(x_linear, z_angular);

        ctre::phoenix::unmanaged::Unmanaged::FeedEnable(10000);

        left_wheel_motor.Set(ControlMode::PercentOutput, d.calculate_wheel_percentOutput()[1]);
        right_wheel_motor.Set(ControlMode::PercentOutput, d.calculate_wheel_percentOutput()[0]);

        RCLCPP_INFO(this->get_logger(), "right_wheel = %0.4f left_wheel = %0.4f", d.calculate_wheel_percentOutput()[0], d.calculate_wheel_percentOutput()[1]);
    }

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr motor_controller_subscriber;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MotorController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}