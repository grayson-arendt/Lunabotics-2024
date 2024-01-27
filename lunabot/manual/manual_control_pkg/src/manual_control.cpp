#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "manual_control_pkg/DifferentialDrive.hpp"

#define Phoenix_No_WPI
#include "ctre/Phoenix.h"
#include "ctre/phoenix/platform/Platform.hpp"
#include "ctre/phoenix/unmanaged/Unmanaged.h"
#include "ctre/phoenix/cci/Unmanaged_CCI.h"

using namespace ctre::phoenix;
using namespace ctre::phoenix::platform;
using namespace ctre::phoenix::motorcontrol;
using namespace ctre::phoenix::motorcontrol::can;

/*
Author: Anthony Baran

Node that subscribes to the joy topic and uses joystick input to move robot. 
*/


std::string interface = "can0";
// create wheel objects of the TalonFX class, pass in the motor ID and interface, default interface is can0
// leaving interface as a parameter for example, in our case we don't need to because the CANbus is named can0
TalonFX left_wheel_motor(2, interface);
TalonFX right_wheel_motor(3);

class ManualControl : public rclcpp::Node
{
public:
    ManualControl() : Node("manual_control")
    {
        // Inverts right motor
        right_wheel_motor.SetInverted(true);

        // Creates a subscriber of type sensor_msgs::msg::Joy, subscribes to the controller_input topic
        // has a que size of 10, and calls callbackControlRobot function
        controller_subscriber = this->create_subscription<sensor_msgs::msg::Joy>(
            "joy", 10,
            std::bind(&ManualControl::callbackControlRobot, this, std::placeholders::_1));
        
        // uses ros2 to output to the terminal
        RCLCPP_INFO(this->get_logger(), " Manual control started. ");
    }

private:
    void callbackControlRobot(const sensor_msgs::msg::Joy::SharedPtr controller_input)
    {
        /*  The joy_node takes data from the xbox controller and publishes it to the controller_input topic. 
            On the topic is a variable named axis. It is an array. 
            axes[0] corresponds to the x axis of the left joystick. You can figure out y. If you want to see this 
            in action, ask me. */
        double x = -1 * (controller_input->axes[0]);
        double y = controller_input->axes[1];
        double r = 1 - controller_input->axes[5];
        r = std::clamp(r, 0.0, 1.0);
        
        // Creating a DifferentialDrive object, see the differential drive header file to understrand how it works
        DifferentialDrive d(x, y, r);

        // FeedEnable needs to be called periodically. It keeps the motors watchdog timers "fed" so they don't shutdown
        // This is a safety mechanism. If it isn't called in 10 seconds, the motor controllers will shutdown.
        ctre::phoenix::unmanaged::Unmanaged::FeedEnable(10000);

        // Setting the percent output 
        left_wheel_motor.Set(ControlMode::PercentOutput, d.calculate_wheel_percentOutput()[1]);
        right_wheel_motor.Set(ControlMode::PercentOutput, d.calculate_wheel_percentOutput()[0]);

        // Uses ros2 to print wheel percent output to the terminal 
        RCLCPP_INFO(this->get_logger(), "right_wheel = %0.4f left_wheel = %0.4f", d.calculate_wheel_percentOutput()[0], d.calculate_wheel_percentOutput()[1]);
    }

    /*  Instantiates a subscriber variable.
        It is a shared pointer of type sensor_msgs::msg::Joy. look into shared pointers if you don't know what they are
        The type is always the same as the data it is subscribing to */
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
