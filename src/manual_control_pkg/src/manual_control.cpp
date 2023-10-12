#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "manual_control_cpp_pkg/DifferentialDrive.hpp"

/*
Author: Anthony Baran

Node that subscribes to the joy topic and uses joystick input to move robot. 
Motor functions still need to be implemented. Right now left and right wheel percent output
are printed to the terminal. 
*/

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

        RCLCPP_INFO(this->get_logger(), " theta = %0.4f right_wheel = %0.4f left_wheel = %0.4f", d.gettheta(), d.calculate_wheel_percentOutput()[0], d.calculate_wheel_percentOutput()[1]);
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