#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <vector>
#include <iostream>
#include <string>
#include <thread>
#include <unistd.h>
#include "ros_phoenix/msg/motor_control.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

//X value from right joystick
float x_val;

class motorPublisher : public rclcpp::Node
{

  //Variables
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub;
  rclcpp::Publisher<ros_phoenix::msg::MotorControl>::SharedPtr motor_pub;
  rclcpp::TimerBase::SharedPtr timer_;
  size_t count_;

  //Creates node with publisher, subscriber, and timer
  public:
    motorPublisher()
    : Node("motor_publisher")
    {
      //Joystick subscriber
      joy_sub = this->create_subscription<sensor_msgs::msg::Joy>("joy", 10, std::bind(&motorPublisher::topic_callback, this, _1));

      //Motor control publisher
      motor_pub = this->create_publisher<ros_phoenix::msg::MotorControl>("back_left/set",1);

      //Timer for publisher
      timer_ = this->create_wall_timer(250ms, std::bind(&motorPublisher::timer_callback, this));
    }

  //Callback for joystick subscriber
  private:
    void topic_callback(const sensor_msgs::msg::Joy::SharedPtr joy_msg) 
    {
      //RCLCPP_INFO(this->get_logger(), "X:'%f', Y: '%f Button1: '%i", joy_msg->axes.at(0), joy_msg->axes.at(1), joy_msg->buttons.at(0));
      x_val = joy_msg->axes.at(0);
    }

  //Callback for motor control publisher
  private:
    void timer_callback()
    {
      auto message = ros_phoenix::msg::MotorControl();

      //Choose mode (output percent)
      message.mode = ros_phoenix::msg::MotorControl::PERCENT_OUTPUT;

      //Set value depending on mode (will send to percent_output)
      message.value = 0.5;

      RCLCPP_INFO(this->get_logger(), "Publishing '%i' as '%f'", message.mode, message.value);
      motor_pub->publish(message);
    }

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<motorPublisher>());
  rclcpp::shutdown();
  return 0;
}