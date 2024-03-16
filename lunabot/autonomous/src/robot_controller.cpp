#include "autonomous/msg/control.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"

#define Phoenix_No_WPI
#include "ctre/Phoenix.h"
#include "ctre/phoenix/cci/Unmanaged_CCI.h"
#include "ctre/phoenix/platform/Platform.hpp"
#include "ctre/phoenix/unmanaged/Unmanaged.h"

using namespace ctre::phoenix;
using namespace ctre::phoenix::platform;
using namespace ctre::phoenix::motorcontrol;
using namespace ctre::phoenix::motorcontrol::can;

TalonFX left_wheel_motor_(1);
TalonFX right_wheel_motor_(2);
TalonFX trencher_motor_(3);
TalonSRX bucket_motor_(4);
TalonSRX actuator_left_motor_(5);
TalonSRX actuator_right_motor_(6);

class RobotController : public rclcpp::Node
{
  public:
    RobotController() : Node("motor_controller")
    {
        right_wheel_motor_.SetInverted(true);

        velocity_subscriber_ = create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", 10, std::bind(&RobotController::callback_velocity, this, std::placeholders::_1));

        control_subscriber_ = create_subscription<autonomous::msg::Control>(
            "control", 10, std::bind(&RobotController::callback_control, this, std::placeholders::_1));

        joystick_subscriber_ = create_subscription<sensor_msgs::msg::Joy>(
            "joy", 10, std::bind(&RobotController::joy_callback, this, std::placeholders::_1));

        declare_and_get_parameters();

        apply_controller_mode();

        manual_enabled_ = true;
        robot_disabled_ = false;

        RCLCPP_INFO(get_logger(), "\033[0;33mMANUAL CONTROL:\033[0m \033[1;32mENABLED\033[0m");
    }

  private:
    void declare_and_get_parameters()
    {
        declare_parameter("xbox_mode", false);
        declare_parameter("ps4_mode", false);
        declare_parameter("switch_mode", false);
        declare_parameter("outdoor_mode", false);

        get_parameter("xbox_mode", xbox_mode_);
        get_parameter("ps4_mode", ps4_mode_);
        get_parameter("switch_mode", switch_mode_);
        get_parameter("outdoor_mode", outdoor_mode_);
    }

    void apply_controller_mode()
    {
        if (xbox_mode_)
        {
            ps4_mode_ = false;
            switch_mode_ = false;
            RCLCPP_INFO(get_logger(), "\033[0;35mXBOX MODE\033[0m");
        }
        else if (ps4_mode_)
        {
            xbox_mode_ = false;
            switch_mode_ = false;
            RCLCPP_INFO(get_logger(), "\033[0;35mPS4 MODE\033[0m");
        }
        else if (switch_mode_)
        {
            xbox_mode_ = false;
            ps4_mode_ = false;
            RCLCPP_INFO(get_logger(), "\033[0;35mSWITCH MODE\033[0m");
        }
        else
        {
            RCLCPP_ERROR(get_logger(), "NO CONTROLLER SELECTED, CHECK LAUNCH PARAMETERS");
        }
    }

    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr joy_msg)
    {
        left_joystick_x_ = joy_msg->axes[0];
        left_joystick_y_ = joy_msg->axes[1];
        left_trigger_ = joy_msg->axes[2];
        right_trigger_ = joy_msg->axes[5];

        d_pad_horizontal_ = switch_mode_ ? joy_msg->axes[4]
                           : ps4_mode_  ? joy_msg->axes[6]
                           : xbox_mode_ ? joy_msg->axes[6]
                                       : 0.0;

        d_pad_vertical_ = switch_mode_ ? joy_msg->axes[5]
                         : ps4_mode_  ? joy_msg->axes[7]
                         : xbox_mode_ ? joy_msg->axes[7]
                                     : 0.0;

        b_button_ = switch_mode_ ? joy_msg->buttons[0]
                   : ps4_mode_  ? joy_msg->buttons[2]
                   : xbox_mode_ ? joy_msg->buttons[1]
                               : -1;

        share_button_ = switch_mode_ ? joy_msg->buttons[9]
                       : ps4_mode_  ? joy_msg->buttons[8]
                       : xbox_mode_ ? joy_msg->buttons[6]
                                   : -1;

        menu_button_ = switch_mode_ ? joy_msg->buttons[10]
                      : ps4_mode_  ? joy_msg->buttons[9]
                      : xbox_mode_ ? joy_msg->buttons[7]
                                  : -1;

        home_button_ = switch_mode_ ? joy_msg->buttons[11]
                      : ps4_mode_  ? joy_msg->buttons[10]
                      : xbox_mode_ ? joy_msg->buttons[8]
                                  : -1;

        speed_multiplier_ = b_button_ ? 1.0 : 0.3;

        if (home_button_)
        {
            robot_disabled_ = true;
        }

        if (share_button_)
        {
            set_manual_enabled(true);
        }

        if (menu_button_)
        {
            set_manual_enabled(false);
        }

        if (switch_mode_ && !ps4_mode_)
        {
            turn_ = left_joystick_x_;
            drive_forward_ = left_joystick_y_;
        }

        else
        {
            turn_ = left_joystick_x_;
            drive_forward_ = (1.0 - right_trigger_) / 2.0;
            drive_backward_ = (1.0 - left_trigger_) / 2.0;
        }

        if (drive_forward_ != 0.0)
        {
            left_power_ = drive_forward_ - turn_;
            right_power_ = drive_forward_ + turn_;
        }

        else if (drive_backward_ != 0.0)
        {
            left_power_ = (drive_backward_ - turn_) * -1.0;
            right_power_ = (drive_backward_ + turn_) * -1.0;
        }

        else
        {
            left_power_ = -turn_;
            right_power_ = turn_;
        }

        actuator_power_ = (d_pad_vertical_ == 1.0) ? 0.5 : (d_pad_vertical_ == -1.0) ? -0.5 : 0.0;
        trencher_power_ = (d_pad_horizontal_ == 1.0) ? 0.5 : (d_pad_horizontal_ == -1.0) ? 0.0 : 0.0;
        bucket_power_ = (d_pad_horizontal_ == -1.0) ? 0.5 : (d_pad_horizontal_ == 1.0) ? 0.0 : 0.0;

  
        if (manual_enabled_)
        {

            if (!robot_disabled_)
            {
                ctre::phoenix::unmanaged::Unmanaged::FeedEnable(100);
            }

            else
            {
                RCLCPP_ERROR(get_logger(), "\033[0;31mROBOT DISABLED\033[0m");
            }

            RCLCPP_DEBUG(get_logger(), "ACTUATOR SPEED %f, BUCKET SPEED: %f, TRENCHER SPEED: %f", actuator_power_,
                    bucket_power_, trencher_power_);

            RCLCPP_DEBUG(get_logger(), "LEFT SPEED: %f, RIGHT SPEED: %f", left_power_ * speed_multiplier_,
                         right_power_ * speed_multiplier_);

            left_wheel_motor_.Set(ControlMode::PercentOutput, left_power_ * speed_multiplier_);
            right_wheel_motor_.Set(ControlMode::PercentOutput, right_power_ * speed_multiplier_);
            trencher_motor_.Set(ControlMode::PercentOutput, trencher_power_);
            
            /* Currently not wired up on robot
            bucket_motor_.Set(ControlMode::PercentOutput, bucket_power_);
            actuator_left_motor_.Set(ControlMode::PercentOutput, actuator_power_);
            actuator_right_motor_.Set(ControlMode::PercentOutput, actuator_power_);
            */
        }
    }

    void callback_velocity(const geometry_msgs::msg::Twist::SharedPtr velocity_msg)
    {
        if (!manual_enabled_)
        {
            ctre::phoenix::unmanaged::Unmanaged::FeedEnable(1000);

            double linear_velocity = velocity_msg->linear.x;
            double angular_velocity = velocity_msg->angular.z;
            double wheel_radius = outdoor_mode_ ? 0.2 : 0.1016;
            double wheel_distance = 0.5;

            double velocity_left_cmd = 0.1 * (linear_velocity - angular_velocity * wheel_distance / 2.0) / wheel_radius;
            double velocity_right_cmd =
                0.1 * (linear_velocity + angular_velocity * wheel_distance / 2.0) / wheel_radius;

            left_wheel_motor_.Set(ControlMode::PercentOutput, velocity_left_cmd);
            right_wheel_motor_.Set(ControlMode::PercentOutput, velocity_right_cmd);
        }
    }

    void callback_control(const autonomous::msg::Control::SharedPtr control_msg)
    {
        manual_enabled_ = control_msg->enable_manual_drive;

        if (control_msg->enable_intake)
        {
            start_mechanism<TalonFX>("TRENCHER", trencher_motor_);
        }

        else if (control_msg->enable_outtake)
        {
            start_mechanism<TalonSRX>("BUCKET", bucket_motor_);
        }

        else if (control_msg->actuator_up)
        {
            start_mechanism<TalonSRX>("ACTUATOR LEFT UP", actuator_left_motor_);
            start_mechanism<TalonSRX>("ACTUATOR RIGHT UP", actuator_right_motor_);
        }

        else if (control_msg->actuator_down)
        {
            start_mechanism<TalonSRX>("ACTUATOR LEFT DOWN", actuator_left_motor_, -0.5);
            start_mechanism<TalonSRX>("ACTUATOR RIGHT DOWN", actuator_right_motor_, -0.5);
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
        RCLCPP_INFO_THROTTLE(get_logger(), clock, 1000, "\033[0;36mAUTONOMOUS CONTROL:\033[0m %s",
                             enabled ? "\033[1;31mDISABLED" : "\033[1;32mENABLED");

        RCLCPP_INFO_THROTTLE(get_logger(), clock, 1000, "\033[0;33mMANUAL CONTROL:\033[0m %s",
                             enabled ? "\033[1;32mENABLED" : "\033[1;31mDISABLED");
        manual_enabled_ = enabled;
    }

  private:
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr velocity_subscriber_;
    rclcpp::Subscription<autonomous::msg::Control>::SharedPtr control_subscriber_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joystick_subscriber_;

    double velocity_left_cmd_, velocity_right_cmd_;
    double left_power_, right_power_;
    double actuator_power_, trencher_power_, bucket_power_;
    double left_trigger_, right_trigger_, d_pad_vertical_, d_pad_horizontal_, left_joystick_x_, left_joystick_y_;
    double turn_, drive_forward_, drive_backward_, speed_multiplier_;
    bool manual_enabled_, robot_disabled_, xbox_mode_, ps4_mode_, switch_mode_, outdoor_mode_;
    bool home_button_, share_button_, menu_button_, b_button_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RobotController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
