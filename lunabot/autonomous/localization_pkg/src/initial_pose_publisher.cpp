#include <string>
#include <utility>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "sensor_msgs/msg/image.hpp"
#include <opencv2/core/mat.hpp>
#include "localization_pkg/AprilTag.h"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"

class InitialPosePublisher : public rclcpp::Node {
public:

    InitialPosePublisher() : Node("initial_pose_publisher") {

        pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("initialpose", 10);
        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>("camera/color/image_raw", rclcpp::QoS(10).reliable(),
        [this](const sensor_msgs::msg::Image::SharedPtr image) {

            if (!tagsDetected) {
                getInitPosition(image, 0.10, 0, 1);
            } else {
                RCLCPP_INFO_ONCE(this->get_logger(), "\033[0;32mTAGS DETECTED\033[0m");
            }
        });
    }

private:

    void getInitPosition(const sensor_msgs::msg::Image::SharedPtr inputImage, double size, int x_id, int y_id) {
        AprilTag x_tag(*inputImage, size, x_id);
        AprilTag y_tag(*inputImage, size, y_id);

        x_tag.detectTag();
        y_tag.detectTag();

        // Keep trying to detect tags until they're both detected
        if ((!x_tag.getDetected() || !y_tag.getDetected()) || (x_tag.z_coord <= 0.0 || y_tag.z_coord <= 0.0)) {
            RCLCPP_INFO(this->get_logger(), "\033[0;35mATTEMPTING TO DETECT BOTH TAGS...\033[0m ");
        }

        else {
            tagsDetected = true;

            auto pose_msg = geometry_msgs::msg::PoseWithCovarianceStamped();
            double x_pos = y_tag.z_coord;
            double y_pos = x_tag.z_coord;
            // Negative because AprilTags are in bottom left corner
            double yaw = (M_PI / 2) - atan2(-y_pos, -x_pos);

            // Normalize the orientation angle to stay within [-pi, pi]
            yaw = std::fmod(yaw + pi, 2 * pi) - pi;

            RCLCPP_INFO_ONCE(this->get_logger(), "\033[0;34mINITIAL POSITION: (%f, %f) (inches)\033[0m", x_pos * 39.370078740156, y_pos * 39.370078740156);
            RCLCPP_INFO_ONCE(this->get_logger(), "\033[0;34mINITIAL ANGLE: %f (radians)\033[0m", yaw);

            tf2::Quaternion q;
            q.setRPY(0, 0, yaw);
            q.normalize();

            pose_msg.header.stamp = this->get_clock()->now();
            pose_msg.header.frame_id = "base_link";
            pose_msg.pose.pose.position.x = x_pos + (0.16 * cos(yaw));
            pose_msg.pose.pose.position.y = y_pos + (0.16 * sin(yaw));
            pose_msg.pose.pose.orientation.x = q.x();
            pose_msg.pose.pose.orientation.y = q.y();
            pose_msg.pose.pose.orientation.z = q.z();
            pose_msg.pose.pose.orientation.w = q.w();
            //pose_msg.ready_to_move = true;

            pose_pub_->publish(pose_msg);
        }
    }

    bool tagsDetected;
    const double pi = 3.141592653589793;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<InitialPosePublisher>());
    rclcpp::shutdown();
    return 0;
}