#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "manual_control_cpp_pkg/DifferentialDrive.hpp"

#define Phoenix_No_WPI
#include "ctre/Phoenix.h"
#include "ctre/phoenix/platform/Platform.hpp"
#include "ctre/phoenix/unmanaged/Unmanaged.h"
#include "ctre/phoenix/cci/Unmanaged_CCI.h"

/*
Author: Anthony Baran

Node that subscribes to the joy topic and uses joystick input to move robot. 
Motor functions still need to be implemented. Right now left and right wheel percent output
are printed to the terminal. 
*/

std::string interface = "can0";
TalonFX left_wheel_motor(1, interface);
TalonFX right_wheel_motor(0, interface);

class ManualControl : public rclcpp::Node
{
public:
    ManualControl() : Node("manual_control")
    {
        controller_subscriber = this->create_subscription<sensor_msgs::msg::Joy>(
            "controller_input", 10,
            std::bind(&ManualControl::callbackControlRobot, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), " Manual control started. ");
    }

private:
    void callbackControlRobot(const sensor_msgs::msg::Joy::SharedPtr controller_input)
    {
        double x = -1 * (controller_input->axes[0]);
        double y = controller_input->axes[1];

        DifferentialDrive d(x, y);

        left_wheel_motor.Set(ControlMode::PercentOutput, d.calculate_wheel_percentOutput()[1]);
        right_wheel_motor.Set(ControlMode::PercentOutput, d.calculate_wheel_percentOutput()[0]);

        RCLCPP_INFO(this->get_logger(), "right_wheel = %0.4f left_wheel = %0.4f", d.calculate_wheel_percentOutput()[0], d.calculate_wheel_percentOutput()[1]);
    }

    void callbackMotorPublisher()
    {

    }

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr controller_subscriber;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ManualControl>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}