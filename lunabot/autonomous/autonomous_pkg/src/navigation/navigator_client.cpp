#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

/**
 * @brief Action client for the Navigation2 navigate_to_pose action server.
 *
 * @author Grayson Arendt
 */
class NavigatorClient : public rclcpp::Node
{

  public:
    using NavigateToPose = nav2_msgs::action::NavigateToPose;
    using GoalHandleNavigate = rclcpp_action::ClientGoalHandle<NavigateToPose>;

    /**
     * @brief Constructor for NavigatorClient.
     */
    NavigatorClient() : Node("navigator_client")
    {
        // Create action client and timer
        this->nav_to_pose_client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");
        this->timer_ =
            this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&NavigatorClient::send_goal, this));
    }

    /**
     * @brief Sends the goal to the action server.
     */
    void send_goal()
    {
        this->timer_->cancel();

        // Check if server is available
        if (!this->nav_to_pose_client_->wait_for_action_server())
        {
            RCLCPP_ERROR(this->get_logger(), "Action server not available... shutting down");
            rclcpp::shutdown();
        }

        // Setting up callbacks and then sending goal to server.
        auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
        send_goal_options.feedback_callback =
            std::bind(&NavigatorClient::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
        send_goal_options.result_callback = std::bind(&NavigatorClient::result_callback, this, std::placeholders::_1);

        // Setting target pose.
        auto goal_msg = NavigateToPose::Goal();
        geometry_msgs::msg::Pose goal_pose;

        // This is just for testing, ideally goal_pose would be an argument for send_goal().
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
    /**
     * @brief Callback for processing feedback from the action server.
     *
     * @param goal_handle The handle to the goal.
     * @param feedback The received feedback.
     */
    void feedback_callback(GoalHandleNavigate::SharedPtr goal_handle,
                           const std::shared_ptr<const NavigateToPose::Feedback> feedback)
    {
        // Update the feedback_callback with correct format strings
        RCLCPP_INFO(this->get_logger(), "Current pose: %d", feedback->navigation_time);
        RCLCPP_INFO(this->get_logger(), "Distance remaining: %d", feedback->distance_remaining);
    }

    /**
     * @brief Callback for processing the result from the action server.
     *
     * @param result The wrapped result.
     */
    void result_callback(const GoalHandleNavigate::WrappedResult &result)
    {
        switch (result.code)
        {
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

        RCLCPP_INFO(this->get_logger(), "Shutting down node now...");
        rclcpp::shutdown();
    }

    rclcpp_action::Client<NavigateToPose>::SharedPtr nav_to_pose_client_;
    rclcpp::TimerBase::SharedPtr timer_;
};

/**
 * @brief Main function.
 *
 * Initializes and spins the NavigatorClient node.
 */
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<NavigatorClient>());
    rclcpp::shutdown();
    return 0;
}
