#include "rclcpp/rclcpp.hpp"
#include "lunabot_autonomous/msg/motor_output.hpp"

#define Phoenix_No_WPI
#include "ctre/phoenix6/TalonFX.hpp"
#include "ctre/phoenix6/Orchestra.hpp"
#include "ctre/phoenix6/unmanaged/Unmanaged.hpp"

namespace phoenix6 = ctre::phoenix6;

using namespace ctre::phoenix6;

class KrakenController : public rclcpp::Node
{
public:
    KrakenController() : Node("kraken_controller")
    {
        motor_output_subscription_ = this->create_subscription<lunabot_autonomous::msg::MotorOutput>(
            "motor_output", 10,
            std::bind(&KrakenController::motorOutputCallback, this, std::placeholders::_1));

        auto& talonFXConfigurator = right_wheel_motor_.GetConfigurator();
        configs::MotorOutputConfigs motorConfigs{};

        motorConfigs.Inverted = signals::InvertedValue::Clockwise_Positive;
        talonFXConfigurator.Apply(motorConfigs);

        orchestra.AddInstrument(left_wheel_motor_);
        orchestra.AddInstrument(right_wheel_motor_);
        orchestra.AddInstrument(vibrator_motor_);
        orchestra.AddInstrument(trencher_motor_);

        orchestra.LoadMusic("/home/intel-nuc/lunabot_ws/src/Lunabotics-2024/lunabot_autonomous/audio/startup.chrp");
        orchestra.Play();
    }

private:
    void motorOutputCallback(const lunabot_autonomous::msg::MotorOutput::SharedPtr msg)
    {
        if (msg->is_enabled)
        {
            ctre::phoenix::unmanaged::FeedEnable(100);

        }

        else
        {
            RCLCPP_ERROR(get_logger(), "\033[0;31mROBOT DISABLED\033[0m");
        }

        // Auto control
        if (!msg->is_manual) {
            velocity_left_output_.Output = msg->left_velocity;
            velocity_right_output_.Output = msg->right_velocity;

            left_wheel_motor_.SetControl(velocity_left_output_);
            right_wheel_motor_.SetControl(velocity_right_output_);
        }

        // Manual control
        else {

            left_output_.Output = msg->left_power;
            right_output_.Output = msg->right_power;
            vibrator_output_.Output = msg->vibrator_power;
            trencher_output_.Output = msg->trencher_power;

            left_wheel_motor_.SetControl(left_output_);
            right_wheel_motor_.SetControl(right_output_);
            trencher_motor_.SetControl(trencher_output_);
            vibrator_motor_.SetControl(vibrator_output_);

        }
    }

    controls::DutyCycleOut velocity_left_output_{0.0};
    controls::DutyCycleOut velocity_right_output_{0.0};
    controls::DutyCycleOut left_output_{0.0};
    controls::DutyCycleOut right_output_{0.0};
    controls::DutyCycleOut trencher_output_{0.0};
    controls::DutyCycleOut vibrator_output_{0.0};

    Orchestra orchestra;
    hardware::TalonFX left_wheel_motor_{1};
    hardware::TalonFX right_wheel_motor_{2};
    hardware::TalonFX vibrator_motor_{5};
    hardware::TalonFX trencher_motor_{6};
    rclcpp::Subscription<lunabot_autonomous::msg::MotorOutput>::SharedPtr motor_output_subscription_;
};


int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<KrakenController>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
