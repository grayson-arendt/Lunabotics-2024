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

class RobotController : public rclcpp::Node
{
public:
    RobotController() : Node("motor_controller")
    {
        right_wheel_motor.SetInverted(true);

        velocity_subscriber_ = create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", 10, std::bind(&RobotController::callback_velocity, this, std::placeholders::_1));

        control_subscriber_ = create_subscription<autonomous::msg::Control>(
            "control", 10, std::bind(&RobotController::callback_control, this, std::placeholders::_1));

        joystick_subscriber_ = create_subscription<sensor_msgs::msg::Joy>(
            "joy", 10, std::bind(&RobotController::joy_callback, this, std::placeholders::_1));

        //RCLCPP_INFO(get_logger(), "\033[0;33mMANUAL CONTROL:\033[0m \033[1;32mENABLED\033[0m");

        robot_disabled = false;
        manual_enabled = false;
        ps4_mode = false;
        outdoor_mode = true;
    }

private:
    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr joy_msg)
    {
        left_joystick = joy_msg->axes[0];
        left_trigger = joy_msg->axes[2];
        right_trigger = joy_msg->axes[5];

        b_button = ps4_mode ? joy_msg->buttons[2] : joy_msg->buttons[1];
        share_button = ps4_mode ? joy_msg->buttons[8] : joy_msg->buttons[6];
        menu_button = ps4_mode ? joy_msg->buttons[9] : joy_msg->buttons[7];
        home_button = ps4_mode ? joy_msg->buttons[10] : joy_msg->buttons[8];

        if (share_button)
        {
            set_manual_enabled(true);
        }

        if (menu_button)
        {
            set_manual_enabled(false);
        }

        if (manual_enabled)
        {
            double turn = left_joystick;
            double drive_forward = (1.0 - right_trigger) / 2.0;
            double drive_backward = (1.0 - left_trigger) / 2.0;
            double speed_multiplier = b_button ? 1.0 : 0.3;

            if (home_button)
                robot_disabled = true;

            if (!robot_disabled)
                ctre::phoenix::unmanaged::Unmanaged::FeedEnable(100);

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

            actuator_power = (actuator_direction == 1.0) ? 0.5 : (actuator_direction == -1.0) ? -0.5 : 0.0;
            bucket_power = (intake_direction == 1.0) ? 0.5 : (intake_direction == -1.0) ? 0.0 : 0.0;
            trencher_power = (intake_direction == -1.0) ? 0.5 : (intake_direction == 1.0) ? 0.0 : 0.0;

            left_wheel_motor.Set(ControlMode::PercentOutput, left_power * speed_multiplier);
            right_wheel_motor.Set(ControlMode::PercentOutput, right_power * speed_multiplier);
        }
    }

    void callback_velocity(const geometry_msgs::msg::Twist::SharedPtr velocity_msg)
    {
        if (!manual_enabled)
        {
            ctre::phoenix::unmanaged::Unmanaged::FeedEnable(1000);

            double linear_velocity = velocity_msg->linear.x;
            double angular_velocity = velocity_msg->angular.z;
            double wheel_radius = outdoor_mode ? 0.2 : 0.1016;
            double wheel_distance = 0.5;

            velocity_left_cmd = 0.1 * (linear_velocity - angular_velocity * wheel_distance / 2.0) / wheel_radius;
            velocity_right_cmd = 0.1 * (linear_velocity + angular_velocity * wheel_distance / 2.0) / wheel_radius;

            left_wheel_motor.Set(ControlMode::PercentOutput, velocity_left_cmd);
            right_wheel_motor.Set(ControlMode::PercentOutput, velocity_right_cmd);
        }
    }

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
            RCLCPP_INFO_THROTTLE(get_logger(), clock, 1000, "\033[0;33mNO MECHANISM ENABLED\033[0m");
        }
    }

    template <typename MotorType>
    void start_mechanism(const std::string &name, MotorType &motor, double percent_output = 0.5)
    {
        RCLCPP_INFO(get_logger(), "\033[0;34m%s:\033[0m \033[1;32mSTARTED\033[0m", name.c_str());
    }

    void set_manual_enabled(bool enabled)
    {
        auto clock = rclcpp::Clock();
        RCLCPP_INFO_THROTTLE(get_logger(), clock, 1000, "\033[0;36mAUTONOMOUS CONTROL:\033[0m %s", enabled ? "\033[1;31mDISABLED" : "\033[1;32mENABLED");
        RCLCPP_INFO_THROTTLE(get_logger(), clock, 1000, "\033[0;33mMANUAL CONTROL:\033[0m %s", enabled ? "\033[1;32mENABLED" : "\033[1;31mDISABLED");
        manual_enabled = enabled;
    }

private:
    double velocity_left_cmd, velocity_right_cmd;
    double left_power, right_power;
    double actuator_power, trencher_power, bucket_power;
    double actuator_direction, intake_direction;
    double left_trigger, right_trigger, left_joystick;
    bool manual_enabled, robot_disabled, ps4_mode, outdoor_mode;
    bool home_button, share_button, menu_button, b_button;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr velocity_subscriber_;
    rclcpp::Subscription<autonomous::msg::Control>::SharedPtr control_subscriber_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joystick_subscriber_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RobotController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
