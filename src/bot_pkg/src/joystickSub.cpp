#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <vector>
#include <iostream>
#include <string>
#include <thread>
#include <unistd.h>

using std::placeholders::_1;

/* 
Author: Grayson Arendt

This program demonstrates how to subscribe to the /joy topic to get joystick and button data.
*/

class JoySubscriber : public rclcpp::Node {

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub;

  public:
    JoySubscriber() : Node("joy_subscriber") {

      // Joystick subscriber
      joy_sub = this->create_subscription<sensor_msgs::msg::Joy>("joy", 10, std::bind(&JoySubscriber::topic_callback, this, _1));
    }

  private:

    // Callback for subscriber
    void topic_callback(const sensor_msgs::msg::Joy::SharedPtr joy_msg) 
    {
      RCLCPP_INFO(this->get_logger(), "X:'%f', Y: '%f Button1: '%i", joy_msg->axes.at(0), joy_msg->axes.at(1), joy_msg->buttons.at(0));
    }
};

int main(int argc, char * argv[]) {
  // Start ROS2
  rclcpp::init(argc, argv);

  // Keep running node
  rclcpp::spin(std::make_shared<JoySubscriber>());

  // Stop ROS2
  rclcpp::shutdown();
  return 0;
}