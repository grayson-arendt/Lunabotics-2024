#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

#define Phoenix_No_WPI
#include "ctre/Phoenix.h"
#include "ctre/phoenix/cci/Unmanaged_CCI.h"
#include "ctre/phoenix/platform/Platform.hpp"
#include "ctre/phoenix/unmanaged/Unmanaged.h"

using namespace ctre::phoenix;
using namespace ctre::phoenix::platform;
using namespace ctre::phoenix::motorcontrol;
using namespace ctre::phoenix::motorcontrol::can;

TalonFX left_wheel_motor(2);
TalonFX right_wheel_motor(3);

/**
 * @brief Converts velocity commands into percent output for motors.
 * @details It also creates an array of encoder values and publishes the previous and current encoder values to a custom
 * Encoder.msg.
 *
 * @author Anthony Baran
 * @author Grayson Arendt
 */
class MotorController : public rclcpp::Node
{

  public:
    /**
     * @brief Constructor for MotorController.
     */
    MotorController() : Node("motor_controller")
    {
        c_FeedEnable(5000);
        right_wheel_motor.SetInverted(true);

        cmd_vel_subscriber = this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", 10, std::bind(&MotorController::callbackMotors, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Motor control started.");
    }

    double velocity_left_cmd;
    double velocity_right_cmd;

    /**
     * @brief Callback function for cmd_vel topic.
     *
     * @param cmd_vel The received Twist message.
     */
    void callbackMotors(const geometry_msgs::msg::Twist::SharedPtr cmd_vel)
    {
        c_FeedEnable(5000);

        double linear_velocity = cmd_vel->linear.x;
        double angular_velocity = cmd_vel->angular.z;

        velocity_left_cmd = linear_velocity - (angular_velocity * (0.2));
        velocity_right_cmd = linear_velocity + (angular_velocity * (0.2));

        left_wheel_motor.Set(ControlMode::PercentOutput, velocity_left_cmd);
        right_wheel_motor.Set(ControlMode::PercentOutput, velocity_right_cmd);
    }

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscriber;
    rclcpp::TimerBase::SharedPtr timer_;
};

/**
 * @brief Main function.
 *
 * Initializes and spins the MotorController node.
 */
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MotorController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
