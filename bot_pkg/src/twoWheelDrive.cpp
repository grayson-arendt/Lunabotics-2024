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
This program subscribes to /joy to get joystick input, 
calculates motor percent_output based off input (equations that allow for turning and driving),
then publishes to each motor's /set topic
*/

using std::placeholders::_1;
using namespace std::chrono_literals;

//X and Y left joystick values
float x_axis;
float y_axis;

class motorPublisher : public rclcpp::Node
{

  //Variables for subscriber and publishers
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub;
  rclcpp::Publisher<ros_phoenix::msg::MotorControl>::SharedPtr L_pub;
  rclcpp::Publisher<ros_phoenix::msg::MotorControl>::SharedPtr R_pub;

  rclcpp::TimerBase::SharedPtr timer_;
  size_t count_;

  //Creates node with publisher, subscriber, and timer
  public:
    motorPublisher()
    : Node("motor_publisher")
    {
      //Joystick subscriber
      joy_sub = this->create_subscription<sensor_msgs::msg::Joy>("joy", 10, std::bind(&motorPublisher::topic_callback, this, _1));

      //Motor control publishers
      L_pub = this->create_publisher<ros_phoenix::msg::MotorControl>("left/set",1);
      R_pub = this->create_publisher<ros_phoenix::msg::MotorControl>("right/set",1);

      //Timer for publisher
      timer_ = this->create_wall_timer(500ms, std::bind(&motorPublisher::timer_callback, this));
    }

  //Callback for joystick subscriber
  private:
    void topic_callback(const sensor_msgs::msg::Joy::SharedPtr joy_msg) 
    {
      //RCLCPP_INFO(this->get_logger(), "X:'%f', Y: '%f Button1: '%i", joy_msg->axes.at(0), joy_msg->axes.at(1), joy_msg->buttons.at(0));

      x_axis = -joy_msg->axes.at(0);
      y_axis = -joy_msg->axes.at(1);
    }

  //Callback for motor control publisher
  private:
    void timer_callback()
    {
      auto left = ros_phoenix::msg::MotorControl();
      auto right = ros_phoenix::msg::MotorControl();
     
      //Choose mode (percent output)
      left.mode = ros_phoenix::msg::MotorControl::PERCENT_OUTPUT;
      right.mode = ros_phoenix::msg::MotorControl::PERCENT_OUTPUT;
    
      //Calculate motor speeds
      float throttle = -y_axis * 0.58;
      float turn = x_axis * 0.58;
      float leftSpeed = throttle - turn;
      float rightSpeed = throttle + turn;

      //Set value depending on mode (will send to output_percent), the multiplication and division is to get rid of extra decimal places
      left.value = round(leftSpeed * 100)/100;
      right.value = round(rightSpeed * 100)/100;

      //This outputs to the terminal
      RCLCPP_INFO(this->get_logger(), "L: '%.2f' R: '%.2f'", left.value, right.value);

      //Publishing to each topics
      L_pub->publish(left);
      R_pub->publish(right);
    }
};

int main(int argc, char * argv[])
{
  //Start node
  rclcpp::init(argc, argv);

  //Keep running node
  rclcpp::spin(std::make_shared<motorPublisher>());

  //Shut down node
  rclcpp::shutdown();
  return 0;
}