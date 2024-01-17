#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include "localization_pkg/PointCloudTo2D.h"

/* Author: Grayson Arendt
 *
 * This program uses a pass-through filter to split the input PointCloud into two separate clouds
 * that use certain height ranges, then combines them and converts it to a LaserScan message.
 */

class PointCloud2DPublisher : public rclcpp::Node {

public:
    PointCloud2DPublisher() : Node("pc_2d_publisher") {
        // PointCloud sub
            camera_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>("camera/depth/color/points", 10, std::bind(&PointCloud2DPublisher::pointCloudCallback, this, std::placeholders::_1));
            cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("cloud_in", 10);
            scan_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>("myScan", 10);
        }


private:

    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {

        boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> rock_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> crater_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> merged_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

        auto new_cloud = sensor_msgs::msg::PointCloud2();

        PointCloudTo2D to2D;

        // Convert to PCL
        pcl::fromROSMsg(*msg, *cloud);

        rock_cloud = to2D.outlierRemoval(cloud, 0.1, 100);
        //rock_cloud = to2D.voxelFilter(rock_cloud, 0.07);
        rock_cloud = to2D.passThroughFilter(rock_cloud, -0.3, 0, "y");

        crater_cloud = to2D.outlierRemoval(cloud, 0.1, 100);
        //crater_cloud = to2D.voxelFilter(crater_cloud, 0.07);
        crater_cloud = to2D.passThroughFilter(crater_cloud, 0.5, 0.8, "y");

        // concatenate
        pcl::concatenate(*rock_cloud, *crater_cloud, *merged_cloud);

        sensor_msgs::msg::LaserScan laser_scan_msg;

        laser_scan_msg.header = msg->header;
        laser_scan_msg.header.frame_id = "camera_link";
        laser_scan_msg.header.stamp = this->now();
        laser_scan_msg.angle_min = -M_PI/2;  // Minimum angle of the laser scan
        laser_scan_msg.angle_max = M_PI/2;   // Maximum angle of the laser scan
        laser_scan_msg.angle_increment = 0.001;  // Angle increment between each measurement
        laser_scan_msg.time_increment = 0.0;  // Time between measurements 
        laser_scan_msg.scan_time = 0.1;       // Time it takes to complete one scan
        laser_scan_msg.range_min = 0.0;       // Minimum range value
        laser_scan_msg.range_max = 10.0;      // Maximum range value

        merged_cloud = to2D.condensePointCloud(rock_cloud);

        laser_scan_msg.ranges.resize(merged_cloud->points.size());
        std::fill(laser_scan_msg.ranges.begin(), laser_scan_msg.ranges.end(), std::numeric_limits<double>::infinity());

        pcl::toROSMsg(*merged_cloud, new_cloud);
        new_cloud.header = msg->header;
        new_cloud.header.frame_id = "camera_link";

        sensor_msgs::PointCloud2ConstIterator<float> iter_x(new_cloud, "x");
        sensor_msgs::PointCloud2ConstIterator<float> iter_y(new_cloud, "y");
        sensor_msgs::PointCloud2ConstIterator<float> iter_z(new_cloud, "z");

        for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
            float x = *iter_x;
            float y = *iter_y;
            float z = *iter_z;

            double range = std::sqrt(x * x + y * y + z * z); // Euclidean distance
            double angle = std::atan2(y, x); // Calculate the angle

            if (range >= laser_scan_msg.range_min && range <= laser_scan_msg.range_max &&
                angle >= laser_scan_msg.angle_min && angle <= laser_scan_msg.angle_max) {
                int index = static_cast<int>((angle - laser_scan_msg.angle_min) / laser_scan_msg.angle_increment);
                if (index >= 0 && index < static_cast<int>(laser_scan_msg.ranges.size())) {
                    laser_scan_msg.ranges[index] = range;
                }
            }
        }

        cloud_pub_->publish(new_cloud);
        scan_pub_->publish(laser_scan_msg);

        RCLCPP_INFO(this->get_logger(), "Publishing merged point cloud...");
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr camera_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;

};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PointCloud2DPublisher>());
    rclcpp::shutdown();
    return 0;
}
