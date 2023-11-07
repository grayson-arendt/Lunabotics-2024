#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

class GoalPublihserTest : public rclcpp::Node
{
public:
    GoalPublihserTest() : Node("test_goal_publisher")
    {
        goal_publisher = this->create_publisher<geometry_msgs::msg::PoseStamped>("goal_pose", 10);
        goal_timer = this->create_wall_timer(std::chrono::seconds(1),
                                             std::bind(&GoalPublihserTest::goal_callback, this));
    }

private:
    void goal_callback()
    {
        auto goal = geometry_msgs::msg::PoseStamped();
        goal.header.frame_id = "map";
        goal.pose.position.x = 4.0;
        goal.pose.position.y = 4.0;
        goal.pose.orientation.w = 1.0;
        goal_publisher->publish(goal);
    }
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_publisher;
    rclcpp::TimerBase::SharedPtr goal_timer;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GoalPublihserTest>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}