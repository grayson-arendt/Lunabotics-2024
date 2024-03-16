#include "autonomous/msg/control.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

/**
 * @brief Sends two goals to the navigation action server as an action client.
 * @details After a goal is reached, this node will publish booleans to the /control
 * topic to enable the specific mechanisms for that goal.
 *
 * @author Grayson Arendt
 */
class NavigatorClient : public rclcpp::Node
{
  public:
    using NavigateToPose = nav2_msgs::action::NavigateToPose;
    using GoalHandleNavigate = rclcpp_action::ClientGoalHandle<NavigateToPose>;

    /**
     * @brief Constructor for NavigatorClient class.
     */
    NavigatorClient()
        : Node("navigator_client"), goal_reached(false), goal_aborted(false), goal_canceled(false),
          navigate_to_excavation(true)
    {
        this->nav_to_pose_client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");
        this->timer_ =
            this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&NavigatorClient::send_goal, this));
        publisher_ = this->create_publisher<autonomous::msg::Control>("control", 10);
    }

    /**
     * @brief Sends the navigation goal.
     */
    void send_goal()
    {
        this->timer_->cancel();

        if (!this->nav_to_pose_client_->wait_for_action_server())
        {
            RCLCPP_ERROR(this->get_logger(), "ACTION SERVER NOT AVAILABLE... SHUTTING DOWN NODE");
            rclcpp::shutdown();
        }

        auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
        send_goal_options.result_callback = std::bind(&NavigatorClient::result_callback, this, std::placeholders::_1);

        auto goal_msg = NavigateToPose::Goal();
        geometry_msgs::msg::Pose goal_pose;

        if (navigate_to_excavation)
        {
            goal_pose.position.x = 0.5;
            goal_pose.position.y = 0.0;
            goal_pose.position.z = 0.0;
            goal_pose.orientation.x = 0.0;
            goal_pose.orientation.y = 0.0;
            goal_pose.orientation.z = 0.0;
            goal_pose.orientation.w = 1.0;
        }
        else
        {
            goal_pose.position.x = 1.0;
            goal_pose.position.y = 0.5;
            goal_pose.position.z = 0.0;
            goal_pose.orientation.x = 0.0;
            goal_pose.orientation.y = 0.0;
            goal_pose.orientation.z = 0.0;
            goal_pose.orientation.w = 1.0;
        }

        goal_msg.pose.pose = goal_pose;
        goal_msg.pose.header.stamp = this->now();
        goal_msg.pose.header.frame_id = "map";

        RCLCPP_INFO(this->get_logger(), "Sending target goal...");
        this->nav_to_pose_client_->async_send_goal(goal_msg, send_goal_options);
    }

  private:
    /**
     * @brief Enables the intake process.
     */
    void enable_intake()
    {
        auto control_msg = autonomous::msg::Control();
        control_msg.enable_intake = true;
        control_msg.enable_outtake = false;
        control_msg.enable_autonomous_drive = false;
        control_msg.enable_manual_drive = false;
        control_msg.actuator_up = false;
        control_msg.actuator_down = false;

        publisher_->publish(control_msg);
    }

    /**
     * @brief Enables the outtake process.
     */
    void enable_outtake()
    {
        auto control_msg = autonomous::msg::Control();
        control_msg.enable_intake = false;
        control_msg.enable_outtake = true;
        control_msg.enable_autonomous_drive = false;
        control_msg.enable_manual_drive = false;
        control_msg.actuator_up = false;
        control_msg.actuator_down = false;

        publisher_->publish(control_msg);
    }

    /**
     * @brief Moves the actuator up.
     */
    void actuator_up()
    {
        auto control_msg = autonomous::msg::Control();
        control_msg.enable_intake = false;
        control_msg.enable_outtake = false;
        control_msg.enable_autonomous_drive = false;
        control_msg.enable_manual_drive = false;
        control_msg.actuator_up = true;
        control_msg.actuator_down = false;

        publisher_->publish(control_msg);
    }

    /**
     * @brief Moves the actuator down.
     */
    void actuator_down()
    {
        auto control_msg = autonomous::msg::Control();
        control_msg.enable_intake = false;
        control_msg.enable_outtake = false;
        control_msg.enable_autonomous_drive = false;
        control_msg.enable_manual_drive = false;
        control_msg.actuator_up = false;
        control_msg.actuator_down = true;

        publisher_->publish(control_msg);
    }

    /**
     * @brief Enables manual drive control.
     */
    void enable_manual_drive()
    {
        auto control_msg = autonomous::msg::Control();
        control_msg.enable_intake = false;
        control_msg.enable_outtake = false;
        control_msg.enable_autonomous_drive = false;
        control_msg.enable_manual_drive = true;
        control_msg.actuator_up = false;
        control_msg.actuator_down = false;

        publisher_->publish(control_msg);
    }

    /**
     * @brief Callback function for result.
     * @param result The result of the navigation goal.
     */
    void result_callback(const GoalHandleNavigate::WrappedResult &result)
    {
        switch (result.code)
        {
        case rclcpp_action::ResultCode::SUCCEEDED:
            goal_reached = true;
            break;
        case rclcpp_action::ResultCode::ABORTED:
            goal_aborted = true;
            break;
        case rclcpp_action::ResultCode::CANCELED:
            goal_canceled = true;
            break;
        default:
            break;
        }

        if (navigate_to_excavation && goal_reached)
        {
            RCLCPP_INFO(this->get_logger(), "\033[1;32mEXCAVATION ZONE REACHED\033[0m");
            this->actuator_down();
            this->enable_intake();
            this->actuator_up();
            navigate_to_excavation = false;
            goal_reached = false;
            send_goal();
        }

        else if (!navigate_to_excavation && goal_reached)
        {
            RCLCPP_INFO(this->get_logger(), "\033[1;32mCONSTRUCTION ZONE REACHED\033[0m");
            this->enable_outtake();
            RCLCPP_INFO(this->get_logger(), "AUTONOMOUS SUCCESS");
        }

        else if ((navigate_to_excavation && goal_aborted) || goal_canceled)
        {
            RCLCPP_INFO(this->get_logger(), "NAVIGATION TO EXCAVATION ZONE FAILED, ENABLING MANUAL CONTROL");
        }

        else if ((!navigate_to_excavation && goal_aborted) || goal_canceled)
        {
            RCLCPP_ERROR(this->get_logger(), "NAVIGATION TO CONSTRUCTION ZONE FAILED, ENABLING MANUAL CONTROL");
            this->enable_manual_drive();
        }

        else
        {
            RCLCPP_ERROR(this->get_logger(), "UKNOWN RESULT CODE, ENABLING MANUAL CONTROL");
            this->enable_manual_drive();
        }
    }

    rclcpp_action::Client<NavigateToPose>::SharedPtr nav_to_pose_client_;
    rclcpp::Publisher<autonomous::msg::Control>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    bool goal_reached, goal_aborted, goal_canceled;
    bool navigate_to_excavation;
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
