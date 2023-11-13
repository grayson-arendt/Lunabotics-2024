#include <algorithm>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "localization_pkg/motor_includes.h"

class MotorController : public rclcpp::Node
{
public:
    MotorController() : Node("motor_controller")
    {        
        right_wheel_motor.SetInverted(true);

        RCLCPP_INFO(this->get_logger(), " Motor control started. ");

        motor_controller_subscriber = this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", 10,
            std::bind(&MotorController::callbackMotors, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), " Motor control started. ");
    }
    double velocity_left_cmd;
    double velocity_right_cmd;

private:

    void callbackMotors(const geometry_msgs::msg::Twist::SharedPtr cmd_vel)
    {
        double linear_velocity = cmd_vel->linear.x;
        double angular_velocity = cmd_vel->angular.z;

        velocity_left_cmd = ((linear_velocity - (angular_velocity * 0.4) / 2.0) / 0.1);

        velocity_right_cmd = ((linear_velocity + (angular_velocity * 0.4) / 2.0) / 0.1);

        velocity_left_cmd = std::clamp(velocity_left_cmd, -0.3, 0.3);
        velocity_right_cmd = std::clamp(velocity_right_cmd, -0.3, 0.3);

        left_wheel_motor.Set(ControlMode::PercentOutput, velocity_left_cmd);
        right_wheel_motor.Set(ControlMode::PercentOutput, velocity_right_cmd);

        RCLCPP_INFO(this->get_logger(), "right_wheel = %0.4f left_wheel = %0.4f", velocity_right_cmd, velocity_left_cmd);
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