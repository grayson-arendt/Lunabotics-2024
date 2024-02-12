#include <algorithm>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "autonomous_msgs/msg/encoder.hpp"

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

/**
 * @brief Converts velocity commands into percent output for motors.
 * @details It also creates an array of encoder values and publishes the previous and current encoder values to a custom Encoder.msg.
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

        left_wheel_motor.ConfigSelectedFeedbackSensor(FeedbackDevice::IntegratedSensor, 0, 10);
        right_wheel_motor.ConfigSelectedFeedbackSensor(FeedbackDevice::IntegratedSensor, 0, 10);

        // Zero the encoders
        left_wheel_motor.SetSelectedSensorPosition(0, 0, 10);
        right_wheel_motor.SetSelectedSensorPosition(0, 0, 10);

        left_encoder_values.push_back(0.0);
        right_encoder_values.push_back(0.0);

        motor_controller_subscriber = this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", 10,
            std::bind(&MotorController::callbackMotors, this, std::placeholders::_1));

        encoder_publisher_ = this->create_publisher<autonomous_msgs::msg::Encoder>("encoders", 10);

        timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&MotorController::encoders_callback, this));

        RCLCPP_INFO(this->get_logger(), "Motor control started.");
        iteration = 0;
    }

    double velocity_left_cmd;
    double velocity_right_cmd;
    double l_initial;
    double r_initial;
    double l_final;
    double r_final;
    int iteration;
    std::vector<double> left_encoder_values;
    std::vector<double> right_encoder_values;

private:
    /**
     * @brief Callback function for the encoders timer.
     */
    void encoders_callback()
    {
        c_FeedEnable(5000);

        auto msg = autonomous_msgs::msg::Encoder();

        left_encoder_values.push_back(left_wheel_motor.GetSelectedSensorPosition());
        right_encoder_values.push_back(right_wheel_motor.GetSelectedSensorPosition());

        msg.left_initial = left_encoder_values[iteration];
        msg.right_initial = right_encoder_values[iteration];
        msg.left_final = left_encoder_values[iteration + 1];
        msg.right_final = right_encoder_values[iteration + 1];

        encoder_publisher_->publish(msg);
        iteration++;
    }

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

        // RCLCPP_INFO(this->get_logger(), "right_wheel = %0.4f left_wheel = %0.4f", velocity_right_cmd, velocity_left_cmd);

        left_wheel_motor.Set(ControlMode::PercentOutput, velocity_left_cmd);
        right_wheel_motor.Set(ControlMode::PercentOutput, velocity_right_cmd);
    }

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr motor_controller_subscriber;
    rclcpp::Publisher<autonomous_msgs::msg::Encoder>::SharedPtr encoder_publisher_;
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
