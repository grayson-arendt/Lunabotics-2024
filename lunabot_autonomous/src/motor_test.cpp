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

/**
 * @brief Tests the motors.
 * @details Uses an Xbox One controller.
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
    }

  private:
    /**
     * @brief Callback function for processing controller input and controlling the motor.
     *
     * @param joy_msg The received Joy message containing controller input.
     */
    void callbackMotors(const sensor_msgs::msg::Joy::SharedPtr joy_msg)
    {
        ctre::phoenix::unmanaged::Unmanaged::FeedEnable(1000);


        d_pad_vertical_ = joy_msg->axes[7];
   

        power = (d_pad_vertical_ == 1.0) ? -0.4 : (d_pad_vertical_ == -1.0) ? 0.4 : 0.0;

   
        left_motor.Set(ControlMode::PercentOutput, power);
        right_motor.Set(ControlMode::PercentOutput, power);

        if (joy_msg->buttons[2]){
          trencher_motor.Set(ControlMode::PercentOutput, -1.0);
        }

        else {
          trencher_motor.Set(ControlMode::PercentOutput, 0.0);
        }
        
    }


    double d_pad_vertical_;
    double power;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr controller_subscriber;
    TalonSRX left_motor{3};
    TalonSRX right_motor{4};
    TalonFX trencher_motor{6};
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
