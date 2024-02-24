#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include <algorithm>

#define Phoenix_No_WPI
#include "ctre/Phoenix.h"
#include "ctre/phoenix/cci/Unmanaged_CCI.h"
#include "ctre/phoenix/platform/Platform.hpp"
#include "ctre/phoenix/unmanaged/Unmanaged.h"

using namespace ctre::phoenix;
using namespace ctre::phoenix::platform;
using namespace ctre::phoenix::motorcontrol;
using namespace ctre::phoenix::motorcontrol::can;

std::string interface = "can0";
TalonFX left_wheel_motor(2, interface);

/**
 * @brief Tests the motors.
 * @details This uses a DualShock4 controller.
 *
 * @author Grayson Arendt
 */
class MotorTest : public rclcpp::Node
{

  public:
    /**
     * @brief Constructor for MotorTest.
     */
    MotorTest() : Node("motor_test")
    {
        controller_subscriber = this->create_subscription<sensor_msgs::msg::Joy>(
            "joy", 10, std::bind(&MotorTest::callbackMotors, this, std::placeholders::_1));

        left_wheel_motor.ConfigFactoryDefault();
        left_wheel_motor.ConfigSelectedFeedbackSensor(FeedbackDevice::IntegratedSensor, 0, 10);
        left_wheel_motor.SetSelectedSensorPosition(0, 0, 10);
    }

  private:
    /**
     * @brief Callback function for processing controller input and controlling the motor.
     *
     * @param controller_input The received Joy message containing controller input.
     */
    void callbackMotors(const sensor_msgs::msg::Joy::SharedPtr controller_input)
    {
        c_FeedEnable(5000);

        if (controller_input->buttons[1])
        {
            left_wheel_motor.Set(ControlMode::PercentOutput, 0.1);
        }

        else
        {
            left_wheel_motor.Set(ControlMode::PercentOutput, 0.0);
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr controller_subscriber;
};

/**
 * @brief Main function.
 *
 * Initializes and spins the MotorTest node.
 */
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MotorTest>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
