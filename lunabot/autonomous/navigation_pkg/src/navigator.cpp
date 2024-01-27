#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

/* 
Author: Grayson Arendt

This program acts as an action client for the Navigation2 navigate to pose action server.
*/

class Navigator : public rclcpp::Node {

public:
    using NavigateToPose = nav2_msgs::action::NavigateToPose;
    using GoalHandleNavigate = rclcpp_action::ClientGoalHandle<NavigateToPose>;
    //using ComputePath = nav2_msgs::action::ComputePathToPose;

    Navigator() : Node("navigator_client") {

        // Create action client and timer
        this->nav_to_pose_client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");
        this->timer_ = this->create_wall_timer(std::chrono::milliseconds(500),std::bind(&Navigator::send_goal, this));
    }

    void send_goal() {
        this->timer_->cancel();

        // Check if server is available
        if (!this->nav_to_pose_client_->wait_for_action_server()) {
            RCLCPP_ERROR(this->get_logger(), "Action server not available... shutting down");
            rclcpp::shutdown();}

        // Setting up callbacks and then sending goal to server
        auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
        send_goal_options.feedback_callback = std::bind(&Navigator::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
        send_goal_options.result_callback = std::bind(&Navigator::result_callback, this, std::placeholders::_1);

        // Setting target pose
        auto goal_msg = NavigateToPose::Goal();
        geometry_msgs::msg::Pose goal_pose;

        // This is just for testing, ideally goal_pose would be an argument for send_goal()
        goal_pose.position.x = 1.0;
        goal_pose.position.y = 0.0;
        goal_pose.position.z = 0.0;

        goal_pose.orientation.x = 0.0;
        goal_pose.orientation.y = 0.0;
        goal_pose.orientation.z = 0.0;
        goal_pose.orientation.w = 1.0;

        goal_msg.pose.pose = goal_pose;
        goal_msg.pose.header.stamp = this->get_clock()->now();
        goal_msg.pose.header.frame_id = "map";

        RCLCPP_INFO(this->get_logger(), "Sending goal pose");
        this->nav_to_pose_client_->async_send_goal(goal_msg, send_goal_options);
    }

private:
    rclcpp_action::Client<NavigateToPose >::SharedPtr nav_to_pose_client_;
    rclcpp::TimerBase::SharedPtr timer_;

    void feedback_callback(GoalHandleNavigate::SharedPtr, const std::shared_ptr<const NavigateToPose::Feedback> feedback) {

        // Update the feedback_callback with correct format strings
        RCLCPP_INFO(this->get_logger(), "Current pose: %f", feedback->navigation_time);
        RCLCPP_INFO(this->get_logger(), "Distance remaining: %f", feedback->distance_remaining);

    }

    void result_callback(const GoalHandleNavigate::WrappedResult & result)
    {
        switch (result.code) {
            case rclcpp_action::ResultCode::SUCCEEDED:
                // If success, have robot do something
                RCLCPP_INFO(this->get_logger(), "Goal reached!");
                break;
            case rclcpp_action::ResultCode::ABORTED:
                RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
                return;
            case rclcpp_action::ResultCode::CANCELED:
                RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
                return;
            default:
                RCLCPP_ERROR(this->get_logger(), "Unknown result code");
                return;
        }

        RCLCPP_INFO(this->get_logger(), "Shutting down node now..." );
        rclcpp::shutdown();
    }
};

int main(int argc, char **argv) {

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Navigator>());
  rclcpp::shutdown();
  return 0;
}
