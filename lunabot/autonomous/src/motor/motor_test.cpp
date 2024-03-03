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

TalonSRX motor(4);

/**
 * @brief Tests the motors
 * @details This uses an Xbox One controller
 *
 * @author Grayson Arendt
 */
class MotorTest : public rclcpp::Node
{

  public:
    /**
     * @brief Constructor for MotorTest
     */
    MotorTest() : Node("motor_test")
    {
        controller_subscriber = this->create_subscription<sensor_msgs::msg::Joy>(
            "joy", 10, std::bind(&MotorTest::callbackMotors, this, std::placeholders::_1));
    }

  private:
    /**
     * @brief Callback function for processing controller input and controlling the motor
     *
     * @param controller_input The received Joy message containing controller input
     */
    void callbackMotors(const sensor_msgs::msg::Joy::SharedPtr controller_input)
    {
        ctre::phoenix::unmanaged::Unmanaged::FeedEnable(1000);

        if (controller_input->buttons[0])
        {
            motor.Set(ControlMode::PercentOutput, 0.8);
        }

        else if (controller_input->buttons[1])
        {
            motor.Set(ControlMode::PercentOutput, -0.8);
        }

        else
        {
            motor.Set(ControlMode::PercentOutput, 0.0);
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr controller_subscriber;
};

/**
 * @brief Main function
 *
 * Initializes and spins the MotorTest node
 */
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MotorTest>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
