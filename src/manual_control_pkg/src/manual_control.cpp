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



std::string interface = "can0";
// create wheel objects of the TalonFX class, pass in the motor ID and interface, default interface is can0
// leaving interface as a parameter for example, in our case we don't need to because the CANbus is named can0
// Get the motor IDs with phoenix tuner and the canable. (Step 2 on jetbrains doc)
TalonSRX left_motor('Insert motor ID here', interface);
TalonSRX right_motor('Insert motor ID here');

class JoystickRead : public rclpp::node
{
    public:
        JoystickReading(): Node("joystick_read")
        {
            //Subscribe to the "joy" topic (Step 4)
            joy_subcrber_ = this->create_subscription<sensor_msgs::msg::Joy>("joy", 10, std::bind(&JoystickRead::joy_callback, this, std::placeholders::_1));

            //Verify that the joystick is connected
            RCLCPP_INFO(this->get_logger(), "Joystick connected");

            private:
                void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg) const
                {
                    //Up and down on the d-pad of a joy controller (Step 5)
                    int up = msg->buttons[12]
                    int down = msg->buttons[13]

                    ctre::phoenix::unmanaged::Unmanaged::FeedEnable(10000);


                    //Print the joystick values to the terminal
                    RCLCPP_INFO(this->get_logger(), "Up d-pad '%f', Down d-pad: '%f'", msg->buttons[12], msg->buttons[13], up, down);
                }

                rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscriber_;
        }
}


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<JoystickRead>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
