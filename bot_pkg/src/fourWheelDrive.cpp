#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <vector>
#include <iostream>
#include <string>
#include <thread>
#include <unistd.h>
#include <math.h>
#include "ros_phoenix/msg/motor_control.hpp"

/*
Author: Grayson Arendt

This program subscribes to /joy to get joystick input, 
calculates motor percent_output based off input (equations that allow for turning and driving),
then publishes to each motor's /set topic
*/

using std::placeholders::_1;
using namespace std::chrono_literals;

// X and Y left joystick values
float x_axis;
float y_axis;

class motorPublisher : public rclcpp::Node
{

  //Variables for subscriber and publishers
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub;
  rclcpp::Publisher<ros_phoenix::msg::MotorControl>::SharedPtr BL_pub;
  rclcpp::Publisher<ros_phoenix::msg::MotorControl>::SharedPtr BR_pub;
  rclcpp::Publisher<ros_phoenix::msg::MotorControl>::SharedPtr FL_pub;
  rclcpp::Publisher<ros_phoenix::msg::MotorControl>::SharedPtr FR_pub;

  rclcpp::TimerBase::SharedPtr timer_;
  size_t count_;

  // Creates node with publisher, subscriber, and timer
  public:
    motorPublisher() : Node("motor_publisher") {

      // Joystick subscriber
      joy_sub = this->create_subscription<sensor_msgs::msg::Joy>("joy", 10, std::bind(&motorPublisher::topic_callback, this, _1));

      // Motor control publishers
      BL_pub = this->create_publisher<ros_phoenix::msg::MotorControl>("back_left/set",1);
      BR_pub = this->create_publisher<ros_phoenix::msg::MotorControl>("back_right/set",1);
      FL_pub = this->create_publisher<ros_phoenix::msg::MotorControl>("front_left/set",1);
      FR_pub = this->create_publisher<ros_phoenix::msg::MotorControl>("front_right/set",1);

      // Timer for publisher
      timer_ = this->create_wall_timer(500ms, std::bind(&motorPublisher::timer_callback, this));
    }

  // Callback for joystick subscriber
  private:
    void topic_callback(const sensor_msgs::msg::Joy::SharedPtr joy_msg) {
      //RCLCPP_INFO(this->get_logger(), "X:'%f', Y: '%f Button1: '%i", joy_msg->axes.at(0), joy_msg->axes.at(1), joy_msg->buttons.at(0));

      x_axis = -joy_msg->axes.at(0);
      y_axis = -joy_msg->axes.at(1);
    }

  // Callback for motor control publisher
  private:
    void timer_callback() {
      auto backLeft = ros_phoenix::msg::MotorControl();
      auto backRight = ros_phoenix::msg::MotorControl();
      auto frontLeft = ros_phoenix::msg::MotorControl();
      auto frontRight = ros_phoenix::msg::MotorControl();

      // Choose mode (percent output)
      backLeft.mode = ros_phoenix::msg::MotorControl::PERCENT_OUTPUT;
      backRight.mode = ros_phoenix::msg::MotorControl::PERCENT_OUTPUT;
      frontLeft.mode = ros_phoenix::msg::MotorControl::PERCENT_OUTPUT;
      frontRight.mode = ros_phoenix::msg::MotorControl::PERCENT_OUTPUT;

      // Calculate motor speeds
      float throttle = -y_axis * 0.58;
      float turn = x_axis * 0.58;
      float leftSpeed = throttle - turn;
      float rightSpeed = throttle + turn;

      // Set value depending on mode (will send to output_percent), the multiplication and division is to get rid of extra decimal places
      backLeft.value = round(leftSpeed * 100)/100;
      backRight.value = round(rightSpeed * 100)/100;
      frontLeft.value = round(leftSpeed * 100)/100;
      frontRight.value = round(rightSpeed * 100)/100;

      // This outputs to the terminal
      RCLCPP_INFO(this->get_logger(), "BL: '%.2f' BR: '%.2f' FL: '%.2f' FR: '%.2f'", backLeft.value, backRight.value, frontLeft.value, frontRight.value);

      // Publishing to each topics
      BL_pub->publish(backLeft);
      BR_pub->publish(backRight);
      FL_pub->publish(frontLeft);
      FR_pub->publish(frontRight);
    }
};

int main(int argc, char * argv[]) {

  //Start ROS2
  rclcpp::init(argc, argv);

  //Keep running node
  rclcpp::spin(std::make_shared<motorPublisher>());

  //Stop ROS2
  rclcpp::shutdown();
  return 0;
}