#include "rclcpp/rclcpp.hpp"
#include "lunabot_autonomous/msg/motor_output.hpp"

#define Phoenix_No_WPI
#include "ctre/Phoenix.h"
#include "ctre/phoenix/platform/Platform.hpp"
#include "ctre/phoenix/unmanaged/Unmanaged.h"

using namespace ctre::phoenix;
using namespace ctre::phoenix::platform;
using namespace ctre::phoenix::motorcontrol;
using namespace ctre::phoenix::motorcontrol::can;

namespace phoenix5 = ctre::phoenix;

class TalonSRXController : public rclcpp::Node
{
public:
    TalonSRXController() : Node("talon_srx_controller")
    {
        motor_output_subscription_ = this->create_subscription<lunabot_autonomous::msg::MotorOutput>(
            "motor_output", 10,
            std::bind(&TalonSRXController::motorOutputCallback, this, std::placeholders::_1));

        magnet_.ConfigFactoryDefault();

		/* setup a basic closed loop */
		magnet_.SetNeutralMode(NeutralMode::Brake);

        /* set the peak and nominal outputs, 12V means full */
        magnet_.ConfigNominalOutputForward(0, 30);
        magnet_.ConfigNominalOutputReverse(0, 30);
        magnet_.ConfigPeakOutputForward(1, 30);
        magnet_.ConfigPeakOutputReverse(-1, 30);

        /* set closed loop gains in slot0 */
        magnet_.Config_kF(0, 0.0, 30);
        magnet_.Config_kP(0, 0.1, 30);
        magnet_.Config_kI(0, 0.001, 30);
        magnet_.Config_kD(0, 0.0, 30);
    }

private:
    void motorOutputCallback(const lunabot_autonomous::msg::MotorOutput::SharedPtr msg)
    {
        if (msg->is_enabled)
        {
            ctre::phoenix::unmanaged::Unmanaged::FeedEnable(100);

        }

        else
        {
            RCLCPP_ERROR(get_logger(), "\033[0;31mROBOT DISABLED\033[0m");
        }

        actuator_left_motor_.Set(ControlMode::PercentOutput, msg->actuator_power);
        actuator_right_motor_.Set(ControlMode::PercentOutput, msg->actuator_power);
        magnet_.Set(ControlMode::Current, msg->magnet_current);
        vibrator_.Set(ControlMode::PercentOutput, msg->vibrator_power);

        //RCLCPP_INFO(get_logger(), "\033[0;35mMAGNET CURRENT: %f\033[0m", magnet_.GetOutputCurrent());
    }

    phoenix5::motorcontrol::can::TalonSRX actuator_left_motor_{3};
    phoenix5::motorcontrol::can::TalonSRX actuator_right_motor_{4};
    phoenix5::motorcontrol::can::TalonSRX magnet_{7};
    phoenix5::motorcontrol::can::TalonSRX vibrator_{8};

    rclcpp::Subscription<lunabot_autonomous::msg::MotorOutput>::SharedPtr motor_output_subscription_;
};


int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TalonSRXController>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
