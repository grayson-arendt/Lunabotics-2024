#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <vector>
#include <iostream>
#include <string>
#include <thread>
#include <unistd.h>
#include "ros_phoenix/msg/motor_control.hpp"

/*
Author: Grayson Arendt

This program publishes PERCENT_OUTPUT as 0.5 to left motor topic as a test.
*/

using std::placeholders::_1;
using namespace std::chrono_literals;

class motorPublisher : public rclcpp::Node {

  // Variables
  rclcpp::Publisher<ros_phoenix::msg::MotorControl>::SharedPtr motor_pub;
  rclcpp::TimerBase::SharedPtr timer_;
  size_t count_;

  // Creates node with publisher, subscriber, and timer
  public:
    motorPublisher() : Node("motor_publisher") {

      //Motor control publisher
      motor_pub = this->create_publisher<ros_phoenix::msg::MotorControl>("left/set",1);

      //Timer for publisher
      timer_ = this->create_wall_timer(250ms, std::bind(&motorPublisher::timer_callback, this));
    }


  //Callback for motor control publisher
  private:
    void timer_callback() {
      auto message = ros_phoenix::msg::MotorControl();

      //Choose mode (output percent)
      message.mode = ros_phoenix::msg::MotorControl::PERCENT_OUTPUT;

      //Set value depending on mode (will send to percent_output)
      message.value = 0.5;

      // Output info to terminal and publish message
      RCLCPP_INFO(this->get_logger(), "Publishing '%i' as '%f'", message.mode, message.value);
      motor_pub->publish(message);
    }
};

int main(int argc, char * argv[]) {
  // Start ROS2
  rclcpp::init(argc, argv);

  // Keep running node
  rclcpp::spin(std::make_shared<motorPublisher>());

  // Stop ROS2
  rclcpp::shutdown();
  return 0;
}