#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

/**
 * @file camera_synchronization.cpp
 * @brief Class for synchronizing camera topics.
 * 
 * This is sometimes necessary for RTABMap or any other program that takes in these camera topics.
 * 
 * @author Grayson Arendt
 */
class CameraSynchronization : public rclcpp::Node
{

public:
    /**
     * @brief Constructor for CameraSynchronization.
     */
    CameraSynchronization() : Node("synchronized_image_camera_node")
    {
        // Image_raw and camera_info subscribers
        rgb_subscriber_.subscribe(this, "/image_raw");
        depth_subscriber_.subscribe(this, "/depth/image_raw");
        info_subscriber_.subscribe(this, "/depth/camera_info");

        // Publishers for synced messages
        synced_rgb_publisher_ = create_publisher<sensor_msgs::msg::Image>("sync_image_raw", 10);
        synced_depth_publisher_ = create_publisher<sensor_msgs::msg::Image>("sync_depth_raw", 10);
        synced_info_publisher_ = create_publisher<sensor_msgs::msg::CameraInfo>("sync_camera_info", 10);

        // Sync image_raw and camera_info messages
        synchronizer_.reset(new message_filters::TimeSynchronizer<sensor_msgs::msg::Image, sensor_msgs::msg::Image, sensor_msgs::msg::CameraInfo>(rgb_subscriber_, depth_subscriber_, info_subscriber_, 10));
        synchronizer_->registerCallback(std::bind(&CameraSynchronization::synchronizedCallback, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
    }

private:
    /**
     * @brief Callback function for synchronized messages
     *
     * @param rgb_msg       Synchronized image message
     * @param depth_msg       Synchronized depth message
     * @param info_msg Synchronized camera_info message
     */
    void synchronizedCallback(const sensor_msgs::msg::Image::ConstSharedPtr &rgb_msg, const sensor_msgs::msg::Image::ConstSharedPtr &depth_msg, const sensor_msgs::msg::CameraInfo::ConstSharedPtr &info_msg)
    {
        /*
        RCLCPP_INFO(
        this->get_logger(),
        "Image Timestamp: %f, Depth Timestamp: %f, CameraInfo Timestamp: %f",
        rgb_msg->header.stamp.sec + rgb_msg->header.stamp.nanosec * 1e-9,
        depth_msg->header.stamp.sec + depth_msg->header.stamp.nanosec * 1e-9,
        info_msg->header.stamp.sec + info_msg->header.stamp.nanosec * 1e-9);
        */

        // Create shared pointers to messages
        auto synced_rgb_msg = std::make_shared<sensor_msgs::msg::Image>(*rgb_msg);
        auto synced_depth_msg = std::make_shared<sensor_msgs::msg::Image>(*depth_msg);
        auto synced_info_msg = std::make_shared<sensor_msgs::msg::CameraInfo>(*info_msg);

        // Set synchronized messages' header timestamps
        synced_rgb_msg->header.stamp = this->get_clock()->now();
        synced_depth_msg->header.stamp = this->get_clock()->now();
        synced_info_msg->header.stamp = this->get_clock()->now();

        // Publish synchronized messages
        synced_rgb_publisher_->publish(*synced_rgb_msg);
        synced_depth_publisher_->publish(*synced_depth_msg);
        synced_info_publisher_->publish(*synced_info_msg);
    }

    message_filters::Subscriber<sensor_msgs::msg::Image> rgb_subscriber_; /**< Subscriber for raw image topic */
    message_filters::Subscriber<sensor_msgs::msg::Image> depth_subscriber_; /**< Subscriber for raw depth image topic */
    message_filters::Subscriber<sensor_msgs::msg::CameraInfo> info_subscriber_; /**< Subscriber for camera_info topic */
    std::shared_ptr<message_filters::TimeSynchronizer<sensor_msgs::msg::Image, sensor_msgs::msg::Image, sensor_msgs::msg::CameraInfo>> synchronizer_; /**< Time synchronizer for image and camera_info messages */
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr synced_rgb_publisher_; /**< Publisher for synchronized image topic */
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr synced_depth_publisher_; /**< Publisher for synchronized depth image topic */
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr synced_info_publisher_; /**< Publisher for synchronized camera_info topic */
};

/**
 * @brief Main function
 *
 * Initializes and spins the CameraSynchronization node
 */
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CameraSynchronization>());
    rclcpp::shutdown();
    return 0;
}
