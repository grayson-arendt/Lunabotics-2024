#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

/* Author: Grayson Arendt
 *
 * This is a simple program that syncs certain D455 camera topics and publishes them as newly synced topics.
 * This is necessary for RTABMap or any other program that takes in these camera topics.
 */

class SynchronizedImageCameraNode : public rclcpp::Node {

public:
    SynchronizedImageCameraNode() : Node("synchronized_image_camera_node") {

        // Image_raw and camera_info subscribers
        image_sub_.subscribe(this, "/image_raw");
        depth_sub_.subscribe(this, "/depth/image_raw");
        camera_info_sub_.subscribe(this, "/depth/camera_info");

        // Publishers for synced messages
        synchronized_image_pub_ = create_publisher<sensor_msgs::msg::Image>("sync_image_raw", 10);
        synchronized_depth_pub_ = create_publisher<sensor_msgs::msg::Image>("sync_depth_raw", 10);
        synchronized_camera_info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>("sync_camera_info", 10);

        // Sync image_raw and camera_info messages
        synchronizer_.reset(new message_filters::TimeSynchronizer<sensor_msgs::msg::Image, sensor_msgs::msg::Image, sensor_msgs::msg::CameraInfo>(image_sub_, depth_sub_, camera_info_sub_, 10));
        synchronizer_->registerCallback(std::bind(&SynchronizedImageCameraNode::synchronizedCallback, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
    }

private:

    void synchronizedCallback(const sensor_msgs::msg::Image::ConstSharedPtr& image_msg, const sensor_msgs::msg::Image::ConstSharedPtr& depth_msg, const sensor_msgs::msg::CameraInfo::ConstSharedPtr& camera_info_msg) {

        /*RCLCPP_INFO(
        this->get_logger(),
        "Image Timestamp: %f, Depth Timestamp: %f, CameraInfo Timestamp: %f",
        image_msg->header.stamp.sec + image_msg->header.stamp.nanosec * 1e-9,
        depth_msg->header.stamp.sec + depth_msg->header.stamp.nanosec * 1e-9,
        camera_info_msg->header.stamp.sec + camera_info_msg->header.stamp.nanosec * 1e-9);
        */

        // Create shared pointers to messages
        auto synced_image_msg = std::make_shared<sensor_msgs::msg::Image>(*image_msg);
        auto synced_depth_msg = std::make_shared<sensor_msgs::msg::Image>(*depth_msg);
        auto synced_camera_info_msg = std::make_shared<sensor_msgs::msg::CameraInfo>(*camera_info_msg);

        // Set synchronized messages' header timestamps
        synced_image_msg->header.stamp = this->get_clock()->now();
        synced_depth_msg->header.stamp = this->get_clock()->now();
        synced_camera_info_msg->header.stamp = this->get_clock()->now();

        // Publish synchronized messages
        synchronized_image_pub_->publish(*synced_image_msg);
        synchronized_depth_pub_->publish(*synced_depth_msg);
        synchronized_camera_info_pub_->publish(*synced_camera_info_msg);
    }

    message_filters::Subscriber<sensor_msgs::msg::Image> image_sub_;
    message_filters::Subscriber<sensor_msgs::msg::Image> depth_sub_;
    message_filters::Subscriber<sensor_msgs::msg::CameraInfo> camera_info_sub_;
    std::shared_ptr<message_filters::TimeSynchronizer<sensor_msgs::msg::Image, sensor_msgs::msg::Image, sensor_msgs::msg::CameraInfo>> synchronizer_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr synchronized_image_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr synchronized_depth_pub_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr synchronized_camera_info_pub_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SynchronizedImageCameraNode>());
    rclcpp::shutdown();
    return 0;
}
