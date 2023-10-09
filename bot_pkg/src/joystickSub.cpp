#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <vector>
#include <iostream>
#include <string>
#include <thread>
#include <unistd.h>

using std::placeholders::_1;


class JoySubscriber : public rclcpp::Node
{
  public:
    JoySubscriber()
    : Node("joy_subscriber")
    {
      joy_sub = this->create_subscription<sensor_msgs::msg::Joy>("joy", 10, std::bind(&JoySubscriber::topic_callback, this, _1));
    }

  private:
    void topic_callback(const sensor_msgs::msg::Joy::SharedPtr joy_msg) 
    {
      RCLCPP_INFO(this->get_logger(), "X:'%f', Y: '%f Button1: '%i", joy_msg->axes.at(0), joy_msg->axes.at(1), joy_msg->buttons.at(0));
    }
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JoySubscriber>());
  rclcpp::shutdown();
  return 0;
}