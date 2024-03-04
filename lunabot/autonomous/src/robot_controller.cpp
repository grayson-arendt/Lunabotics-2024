#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "autonomous/msg/control.hpp"

#define Phoenix_No_WPI
#include "ctre/Phoenix.h"
#include "ctre/phoenix/cci/Unmanaged_CCI.h"
#include "ctre/phoenix/platform/Platform.hpp"
#include "ctre/phoenix/unmanaged/Unmanaged.h"

using namespace ctre::phoenix;
using namespace ctre::phoenix::platform;
using namespace ctre::phoenix::motorcontrol;
using namespace ctre::phoenix::motorcontrol::can;

TalonFX left_wheel_motor(1);
TalonFX right_wheel_motor(2);
TalonFX trencher_motor(3);
TalonSRX actuator_motor(4);
TalonSRX bucket_motor(5);

/**
 * @brief Node for controlling robot. Has both autonomous and manual control
 * @details
 * Settings/menu button -> Disables autonomous control and enables manual control
 * Home button -> Disables all motors in manual control
 *
 * Button layout for manual control:
 *
 * Left trigger -> Backwards speed
 * Right trigger -> Forwards speed
 * B button -> Turbo mode
 * D-pad up -> Linear actuator up
 * D-pad down -> Linear actuator down
 * D-pad right -> Trencher intake
 * D-pad left -> Bucket outtake
 *
 * @author Grayson Arendt
 */

class RobotController : public rclcpp::Node
{
public:
    /**
     * @brief Constructor for RobotController class
     */
    RobotController() : Node("motor_controller")
    {
        right_wheel_motor.SetInverted(true);

        velocity_subscriber_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", 10, std::bind(&RobotController::callback_velocity, this, std::placeholders::_1));

        control_subscriber_ = this->create_subscription<autonomous::msg::Control>(
            "control", 10, std::bind(&RobotController::callback_control, this, std::placeholders::_1));

        joystick_subscriber_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "joy", 10, std::bind(&RobotController::joy_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "\033[0;35mAUTONOMOUS CONTROL:\033[0m \033[1;32mENABLED\033[0m");

        robot_disabled = false;
        manual_enabled = false;
    }

private:
    /**
     * @brief Callback function for processing controller input and manually controlling robot
     *
     * @param joy_msg The received Joy message containing controller input
     */
    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr joy_msg)
    {
        if (joy_msg->buttons[6])
        {
            auto clock = rclcpp::Clock();
            RCLCPP_INFO_THROTTLE(this->get_logger(), clock, 1000, "\033[0;36mAUTONOMOUS CONTROL:\033[0m \033[1;32mENABLED\033[0m");
            RCLCPP_INFO_THROTTLE(this->get_logger(), clock, 1000, "\033[0;33mMANUAL CONTROL:\033[0m \033[1;31mDISABLED\033[0m");
            manual_enabled = false;
        }

        if (joy_msg->buttons[7])
        {
            auto clock = rclcpp::Clock();
            RCLCPP_INFO_THROTTLE(this->get_logger(), clock, 1000, "\033[0;36mAUTONOMOUS CONTROL:\033[0m \033[1;31mDISABLED\033[0m");
            RCLCPP_INFO_THROTTLE(this->get_logger(), clock, 1000, "\033[0;33mMANUAL CONTROL:\033[0m \033[1;32mENABLED\033[0m");
            manual_enabled = true;
        }

        if (manual_enabled)
        {
            double turn = joy_msg->axes[0];
            double drive_forward = (1.0 - joy_msg->axes[5]) / 2.0;
            double drive_backward = (1.0 - joy_msg->axes[2]) / 2.0;
            double speed_multiplier = joy_msg->buttons[1] ? 1.0 : 0.3;

            if (joy_msg->buttons[8])
            {
                RCLCPP_ERROR_ONCE(this->get_logger(), "\033[1;31mROBOT DISABLED\033[0m");
                robot_disabled = true;
            }

            if (!robot_disabled)
            {
                ctre::phoenix::unmanaged::Unmanaged::FeedEnable(100);
            }

            if (drive_forward != 0.0)
            {
                left_power = drive_forward - turn;
                right_power = drive_forward + turn;
            }

            else if (drive_backward != 0.0)
            {
                left_power = (drive_backward - turn) * -1.0;
                right_power = (drive_backward + turn) * -1.0;
            }

            else
            {
                left_power = -turn;
                right_power = turn;
            }

            /*
            if (actuator_direction == 1.0)
            {
                actuator_power = 0.5;
            }
            else if (actuator_direction == -1.0)
            {
                actuator_power = -0.5;
            }
            else
            {
                actuator_power = 0.0;
            }

            if (intake_direction == 1.0)
            {
                bucket_power = 0.5;
                trencher_power = 0.0;
            }
            else if (intake_direction == -1.0)
            {
                trencher_power = 0.5;
                bucket_power = 0.0;
            }
            else
            {
                bucket_power = 0.0;
                trencher_power = 0.0;
            }
            */

            left_wheel_motor.Set(ControlMode::PercentOutput, left_power * speed_multiplier);
            right_wheel_motor.Set(ControlMode::PercentOutput, right_power * speed_multiplier);

            /*
            trencher_motor.Set(ControlMode::PercentOutput, trencher_power);
            actuator_motor.Set(ControlMode::PercentOutput, actuator_power);
            bucket_motor.Set(ControlMode::PercentOutput, bucket_power);
            */
        }
    }

    /**
     * @brief Callback function for processing velocity commands
     *
     * @param velocity_msg The received Twist message containing velocity
     */
    void callback_velocity(const geometry_msgs::msg::Twist::SharedPtr velocity_msg)
    {
        if (!manual_enabled)
        {
            ctre::phoenix::unmanaged::Unmanaged::FeedEnable(100);

            double linear_velocity = velocity_msg->linear.x;
            double angular_velocity = velocity_msg->angular.z;

            velocity_left_cmd = linear_velocity - (angular_velocity * (0.2));
            velocity_right_cmd = linear_velocity + (angular_velocity * (0.2));

            left_wheel_motor.Set(ControlMode::PercentOutput, velocity_left_cmd);
            right_wheel_motor.Set(ControlMode::PercentOutput, velocity_right_cmd);
        }
    }

    /**
     * @brief Callback function for activating digging or depositing mechanisms based off of /control topic
     *
     * @param control_msg The received Control message containing booleans to enable specific mechanisms
     */
    void callback_control(const autonomous::msg::Control::SharedPtr control_msg)
    {
        manual_enabled = control_msg->enable_manual_drive;

        if (control_msg->enable_intake)
        {
            start_mechanism<TalonFX>("TRENCHER", trencher_motor);
        }

        else if (control_msg->enable_outtake)
        {
            start_mechanism<TalonSRX>("BUCKET", bucket_motor);
        }

        else if (control_msg->actuator_up)
        {
            start_mechanism<TalonSRX>("ACTUATOR UP", actuator_motor);
        }

        else if (control_msg->actuator_down)
        {
            start_mechanism<TalonSRX>("ACTUATOR DOWN", actuator_motor, -0.5);
        }

        else
        {
            auto clock = rclcpp::Clock();
            RCLCPP_INFO_THROTTLE(this->get_logger(), clock, 1000, "\033[0;33mNO MECHANISM ENABLED\033[0m");
        }
    }

    /**
     * @brief Callback function for starting mechanisms
     *
     * @param name The name of mechanism
     * @param motor The motor associated with the mechanism
     * @param percent_output The percent output speed of the mechanism
     */

    template <typename MotorType>
    void start_mechanism(const std::string &name, MotorType &motor, double percent_output = 0.5)
    {
        RCLCPP_INFO(this->get_logger(), "\033[0;34m%s:\033[0m \033[1;32mSTARTED\033[0m", name.c_str());

        /*
        Will use encoders once they are wired up. This is just for testing.

        motor.Set(ControlMode::PercentOutput, percent_output);

        // Run for 5 seconds then turn off
        for (int i = 0; i < 5; i++) {
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }

        motor.Set(ControlMode::PercentOutput, 0.0);
        std::this_thread::sleep_for(std::chrono::seconds(1));
        */
    }

private:
    double velocity_left_cmd, velocity_right_cmd;
    double left_power, right_power;
    double actuator_power, trencher_power, bucket_power;
    double actuator_direction, intake_direction;
    bool manual_enabled, autonomous_disabled, robot_disabled;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr velocity_subscriber_;
    rclcpp::Subscription<autonomous::msg::Control>::SharedPtr control_subscriber_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joystick_subscriber_;
};

/**
 * @brief Main function.
 *
 * Initializes and spins the RobotController node
 */
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RobotController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
