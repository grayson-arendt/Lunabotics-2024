#include "rclcpp/rclcpp.hpp"
 
class GoalPublihserTest : public rclcpp::Node 
{
public:
    GoalPublihserTest() : Node("test_goal_publisher") 
    {
    }
 
private:
};
 
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GoalPublihserTest>(); 
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}