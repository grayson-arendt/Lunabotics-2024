#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

class SynchronizedImageCameraNode : public rclcpp::Node {
public:
    SynchronizedImageCameraNode() : Node("synchronized_image_camera_node") {

        // Image_raw and camera_info subscribers
        image_sub_.subscribe(this, "/camera/color/image_raw");
        camera_info_sub_.subscribe(this, "/camera/color/camera_info");

        // Publishers for synced messages
        synchronized_image_pub_ = create_publisher<sensor_msgs::msg::Image>("sync_image_raw", 10);
        synchronized_camera_info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>("sync_camera_info", 10);

        // Sync image_raw and camera_info messages
        synchronizer_.reset(new message_filters::TimeSynchronizer<sensor_msgs::msg::Image, sensor_msgs::msg::CameraInfo>(image_sub_, camera_info_sub_, 10));
        synchronizer_->registerCallback(std::bind(&SynchronizedImageCameraNode::synchronizedCallback, this, std::placeholders::_1, std::placeholders::_2));
    }

private:
    message_filters::Subscriber<sensor_msgs::msg::Image> image_sub_;
    message_filters::Subscriber<sensor_msgs::msg::CameraInfo> camera_info_sub_;
    std::shared_ptr<message_filters::TimeSynchronizer<sensor_msgs::msg::Image, sensor_msgs::msg::CameraInfo>> synchronizer_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr synchronized_image_pub_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr synchronized_camera_info_pub_;

    void synchronizedCallback(const sensor_msgs::msg::Image::ConstSharedPtr& image_msg, const sensor_msgs::msg::CameraInfo::ConstSharedPtr& camera_info_msg) {

        // Publish synchronized image_raw message
        RCLCPP_INFO(this->get_logger(), "Publishing synced messages...");
        synchronized_image_pub_->publish(*image_msg);

        // Publish synchronized camera_info message
        synchronized_camera_info_pub_->publish(*camera_info_msg);
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SynchronizedImageCameraNode>());
    rclcpp::shutdown();
    return 0;
}
