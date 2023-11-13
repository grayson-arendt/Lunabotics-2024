#include <algorithm>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"

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

/*
This uses a DualShock4 controller
*/

class MotorTest : public rclcpp::Node
{
public:
    MotorTest() : Node("motor_test")
    {
        controller_subscriber = this->create_subscription<sensor_msgs::msg::Joy>("joy", 10,
        std::bind(&MotorTest::callbackMotors, this, std::placeholders::_1));

        left_wheel_motor.ConfigFactoryDefault();
        left_wheel_motor.ConfigSelectedFeedbackSensor(FeedbackDevice::IntegratedSensor, 0, 10);
        left_wheel_motor.SetSelectedSensorPosition(0, 0, 10);

    }

private:
    void callbackMotors(const sensor_msgs::msg::Joy::SharedPtr controller_input)
    {   
        c_FeedEnable(5000);

        //zero encoders
        if(controller_input->buttons[1])
        {
            /* Zero the sensor */
            left_wheel_motor.SetSelectedSensorPosition(0, 0, 10);
        }

        else if (controller_input->buttons[3]) {
            left_wheel_motor.Set(ControlMode::PercentOutput, 0.1);
            RCLCPP_INFO(this->get_logger(), "Encoder Pos: %f", left_wheel_motor.GetSelectedSensorPosition());
        }

        else {
            left_wheel_motor.Set(ControlMode::PercentOutput, 0.0);
        }



        //double leftYstick = controller_input->axes[1];
        //RCLCPP_INFO(this->get_logger(), "Left Y: %f, Encoder Pos: %f", leftYstick, left_wheel_motor.GetSelectedSensorPosition());
        //left_wheel_motor.Set(ControlMode::PercentOutput, leftYstick);
    }

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr controller_subscriber;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MotorTest>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}