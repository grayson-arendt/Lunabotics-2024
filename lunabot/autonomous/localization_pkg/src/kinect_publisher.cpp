#include <string.h>
#include <libfreenect.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include "rclcpp/rclcpp.hpp"
#include "camera_info_manager/camera_info_manager.hpp"
#include "image_transport/image_transport.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "cv_bridge/cv_bridge.h"

using namespace std::chrono_literals;

static cv::Mat _depth_image(cv::Mat::zeros(cv::Size(640, 480), CV_16UC1));
static cv::Mat _rgb_image(cv::Mat::zeros(cv::Size(640, 480), CV_8UC3));

class KinectPublisher : public rclcpp::Node {
public:
  KinectPublisher() : Node("kinect_publisher") {
    freenect_context *f_ctx;
    freenect_device *f_dev;
    int die = 0;

    printf("Kinect camera test\n");

    if (freenect_init(&f_ctx, NULL) < 0) {
      printf("freenect_init() failed\n");
      return;
    }

    if (freenect_open_device(f_ctx, &f_dev, 0) < 0) {
      printf("Could not open device\n");
      return;
    }

    freenect_set_depth_callback(f_dev, depth_cb);
    freenect_set_video_callback(f_dev, rgb_cb);

    freenect_start_depth(f_dev);
    freenect_start_video(f_dev);

    rgb_info_manager_ = std::make_shared<camera_info_manager::CameraInfoManager>(
        this, "kinect",
        "file:///path/to/calibration_rgb.yaml");
    depth_info_manager_ = std::make_shared<camera_info_manager::CameraInfoManager>(
        this, "kinect",
        "file:///path/to/calibration_depth.yaml");

    rgb_info_ = rgb_info_manager_->getCameraInfo();
    rgb_info_.header.frame_id = "camera_link";
    depth_info_ = depth_info_manager_->getCameraInfo();
    depth_info_.header.frame_id = "camera__link";

    depth_pub_ = image_transport::create_camera_publisher(this, "depth/image_raw");
    rgb_pub_ = image_transport::create_camera_publisher(this, "image_raw");

    timer_ = create_wall_timer(16ms, std::bind(&KinectPublisher::timer_callback, this));
    freenect_process_events(f_ctx);
    fn_ctx_ = f_ctx;
    fn_dev_ = f_dev;
  }

private:

    static void depth_cb(freenect_device *dev, void *depth, uint32_t timestamp) {
        _depth_image = cv::Mat(480, 640, CV_16UC1, depth);
    }

    static void rgb_cb(freenect_device *dev, void *rgb, uint32_t timestamp) {
        _rgb_image = cv::Mat(480, 640, CV_8UC3, rgb);
     }

    void timer_callback() {

        auto common_stamp = now();
        auto depth_header = std_msgs::msg::Header();
        auto rgb_header = std_msgs::msg::Header();

        depth_header.frame_id = "camera_link";
        rgb_header.frame_id = "camera_link";

        depth_header.stamp = common_stamp;
        depth_info_.header.stamp = common_stamp;

        rgb_header.stamp = common_stamp;
        rgb_info_.header.stamp = common_stamp;

        auto depth_msg = cv_bridge::CvImage(depth_header, "16UC1", _depth_image).toImageMsg();
        auto rgb_msg = cv_bridge::CvImage(rgb_header, "rgb8", _rgb_image).toImageMsg();

        depth_pub_.publish(*depth_msg, depth_info_);
        rgb_pub_.publish(*rgb_msg, rgb_info_);
}


freenect_context *fn_ctx_;
freenect_device *fn_dev_;
rclcpp::TimerBase::SharedPtr timer_;
std::shared_ptr<camera_info_manager::CameraInfoManager> rgb_info_manager_, depth_info_manager_;
sensor_msgs::msg::CameraInfo rgb_info_, depth_info_;
image_transport::CameraPublisher depth_pub_, rgb_pub_;

};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<KinectPublisher>());
  rclcpp::shutdown();
  return 0;
}