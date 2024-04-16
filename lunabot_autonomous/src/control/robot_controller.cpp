#include "lunabot_autonomous/msg/control.hpp"
#include "lunabot_autonomous/msg/motor_output.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"

/**
 * @class RobotController
 * @brief A class for controlling the robot using a controller 
 * (XBox One, PS4, or Nintendo Switch) and autonomous commands.
 * @details 
 * The Nintendo Switch controller triggers act as buttons instead 
 * of providing variable input, so the mode will drive the robot
 * based off of the left joystick instead.
 * 
 * @author Grayson Arendt
 */
class RobotController : public rclcpp::Node
{
  public:
    /**
     * @brief Constructor for RobotController.
     */
    RobotController() : Node("motor_controller")
    {
        velocity_subscriber_ = create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", 10, std::bind(&RobotController::callback_velocity, this, std::placeholders::_1));

        joystick_subscriber_ = create_subscription<sensor_msgs::msg::Joy>(
            "joy", 10, std::bind(&RobotController::joy_callback, this, std::placeholders::_1));

        motor_output_publisher_ = this->create_publisher<lunabot_autonomous::msg::MotorOutput>("motor_output", 10);

        declare_and_get_parameters();
        apply_controller_mode();

        manual_enabled_ = true;
        robot_disabled_ = false;

        RCLCPP_INFO(get_logger(), "\033[0;33mMANUAL CONTROL:\033[0m \033[1;32mENABLED\033[0m");
    }

  private:
    /**
     * @brief Declares and gets parameters from the parameter server.
     */
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

    /**
     * @brief Applies the selected controller mode.
     */
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

    /**
     * @brief Callback function for processing joystick messages.
     * @param joy_msg The joystick message.
     */
    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr joy_msg)
    {

        share_button_ = switch_mode_ ? joy_msg->buttons[9]
                        : ps4_mode_  ? joy_msg->buttons[8]
                        : xbox_mode_ ? joy_msg->buttons[6]
                                     : -1;

        menu_button_ = switch_mode_ ? joy_msg->buttons[10]
                       : ps4_mode_  ? joy_msg->buttons[9]
                       : xbox_mode_ ? joy_msg->buttons[7]
                                    : -1;

        if (share_button_)
        {
            auto clock = rclcpp::Clock();
            RCLCPP_INFO_THROTTLE(get_logger(), clock, 1000,
                                 "\033[0;36mAUTONOMOUS CONTROL: \033[0m\033[1;31mDISABLED\033[0m");
            RCLCPP_INFO_THROTTLE(get_logger(), clock, 1000,
                                 "\033[0;33mMANUAL CONTROL: \033[0m\033[1;32mENABLED\033[0m");
            manual_enabled_ = true;
        }

        if (menu_button_)
        {
            auto clock = rclcpp::Clock();
            RCLCPP_INFO_THROTTLE(get_logger(), clock, 1000,
                                 "\033[0;36mAUTONOMOUS CONTROL: \033[0m\033[1;32mENABLED\033[0m");
            RCLCPP_INFO_THROTTLE(get_logger(), clock, 1000,
                                 "\033[0;33mMANUAL CONTROL: \033[0m\033[1;31mDISABLED\033[0m");
            manual_enabled_ = false;
        }

        if (manual_enabled_)
        {
            left_joystick_x_ = joy_msg->axes[0];
            left_joystick_y_ = joy_msg->axes[1];
            left_trigger_ = joy_msg->axes[2];
            right_trigger_ = joy_msg->axes[5];

            home_button_ = switch_mode_ ? joy_msg->buttons[11]
                           : ps4_mode_  ? joy_msg->buttons[10]
                           : xbox_mode_ ? joy_msg->buttons[8]
                                        : -1;

            d_pad_horizontal_ = switch_mode_ ? joy_msg->axes[4]
                                : ps4_mode_  ? joy_msg->axes[6]
                                : xbox_mode_ ? joy_msg->axes[6]
                                             : 0.0;

            d_pad_vertical_ = switch_mode_ ? joy_msg->axes[5]
                              : ps4_mode_  ? joy_msg->axes[7]
                              : xbox_mode_ ? joy_msg->axes[7]
                                           : 0.0;

            a_button_ = switch_mode_ ? joy_msg->buttons[1]
                        : ps4_mode_  ? joy_msg->buttons[0]
                        : xbox_mode_ ? joy_msg->buttons[1]
                                     : -1;
            
            b_button_ = switch_mode_ ? joy_msg->buttons[0]
                        : ps4_mode_  ? joy_msg->buttons[1]
                        : xbox_mode_ ? joy_msg->buttons[2]
                                     : -1;

            x_button_ = switch_mode_ ? joy_msg->buttons[2]
                        : ps4_mode_  ? joy_msg->buttons[3]
                        : xbox_mode_ ? joy_msg->buttons[1]
                                     : -1;
            
            y_button_ = switch_mode_ ? joy_msg->buttons[3]
                        : ps4_mode_  ? joy_msg->buttons[2]
                        : xbox_mode_ ? joy_msg->buttons[1]
                                     : -1;

            trencher_power_ = a_button_ ? -0.6 : 0.0;
            vibrator_power_ = x_button_ ? 0.1 : y_button_ ? -0.1 : 0.0;
            actuator_power_ = (d_pad_vertical_ == 1.0) ? -0.3 : (d_pad_vertical_ == -1.0) ? 0.3 : 0.0;
	        magnet_power_ = b_button_ ? 0.0 : 0.3;
            speed_multiplier_ = 0.45;

            if (home_button_)
            {
                robot_disabled_ = true;
            }

            if (switch_mode_)
            {
                turn_ = left_joystick_x_;
                drive_ = left_joystick_y_;

                left_power_ = drive_ - turn_;
                right_power_ = drive_ + turn_;
            }
            else
            {
                turn_ = left_joystick_x_;
                drive_forward_ = (1.0 - right_trigger_) / 2.0;
                drive_backward_ = (1.0 - left_trigger_) / 2.0;

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
            }

            if (robot_disabled_) {
                motor_output_msg_.is_enabled = false;
            }

            else {
                motor_output_msg_.is_enabled = true;
            }

            motor_output_msg_.is_manual = true;
            motor_output_msg_.left_power = left_power_ * speed_multiplier_;
            motor_output_msg_.right_power = right_power_ * speed_multiplier_;
            motor_output_msg_.trencher_power = trencher_power_;
            motor_output_msg_.vibrator_power = vibrator_power_;
            motor_output_msg_.actuator_power = actuator_power_;
            motor_output_msg_.magnet_power = magnet_power_;

            motor_output_publisher_->publish(motor_output_msg_);
        }
    }

    /**
     * @brief Callback function for processing velocity messages.
     * @param velocity_msg The velocity message.
     */
    void callback_velocity(const geometry_msgs::msg::Twist::SharedPtr velocity_msg)
    {
        if (!manual_enabled_)
        {
            double linear_velocity = velocity_msg->linear.x;
            double angular_velocity = velocity_msg->angular.z;
            double wheel_radius = outdoor_mode_ ? 0.2 : 0.1016;
            double wheel_distance = 0.5;

            velocity_left_cmd_ = 0.1 * (linear_velocity - angular_velocity * wheel_distance / 2.0) / wheel_radius;
            velocity_right_cmd_ = 0.1 * (linear_velocity + angular_velocity * wheel_distance / 2.0) / wheel_radius;

            motor_output_msg_.is_manual = false;
            motor_output_msg_.left_velocity = velocity_left_cmd_;
            motor_output_msg_.right_velocity = velocity_right_cmd_;

            motor_output_publisher_->publish(motor_output_msg_);
        }
    }

  private:
    double velocity_left_cmd_, velocity_right_cmd_, left_power_, right_power_, trencher_power_, vibrator_power_, actuator_power_, magnet_power_;
    double left_trigger_, right_trigger_, d_pad_vertical_, d_pad_horizontal_, left_joystick_x_, left_joystick_y_;
    double turn_, drive_, drive_forward_, drive_backward_, speed_multiplier_, trencher_speed_multiplier_;
    bool manual_enabled_, robot_disabled_, xbox_mode_, ps4_mode_, switch_mode_, outdoor_mode_;
    bool home_button_, share_button_, menu_button_, a_button_, b_button_, x_button_, y_button_;

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr velocity_subscriber_;
    rclcpp::Subscription<lunabot_autonomous::msg::Control>::SharedPtr control_subscriber_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joystick_subscriber_;
    rclcpp::Publisher<lunabot_autonomous::msg::MotorOutput>::SharedPtr motor_output_publisher_;
    lunabot_autonomous::msg::MotorOutput motor_output_msg_;
};

/**
 * @brief Main function.
 *
 * Initializes and spins the RobotController node.
 */
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RobotController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
