#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

class GoalPublihserTest : public rclcpp::Node
{
public:
    GoalPublihserTest() : Node("test_goal_publisher")
    {
        goal_publisher = this->create_publisher<geometry_msgs::msg::PoseStamped>("goal_pose", 10);
    }
    void goal()
    {
        auto goal = geometry_msgs::msg::PoseStamped();
        goal.header.frame_id = "map";
        goal.pose.position.x = 4.0;
        goal.pose.position.y = 4.0;
        goal.pose.orientation.w = 1.0;
        goal_publisher->publish(goal);
    }
private:
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_publisher;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GoalPublihserTest>();
    node->goal();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}