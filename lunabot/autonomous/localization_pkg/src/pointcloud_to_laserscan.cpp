#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

/* Author: Grayson Arendt
 *
 * This program converts a PointCloud2 message into a LaserScan message.
 */

class PointCloudToLaserScan : public rclcpp::Node {

public:
    PointCloudToLaserScan() : Node("pointcloud_to_laserscan") {
            cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>("rtabmap/cloud_obstacles", 10, std::bind(&PointCloudToLaserScan::cloudCallback, this, std::placeholders::_1));
            scan_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>("scan2", 10);
    }

private:
    void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr cloud) {

        auto laser_scan_msg = sensor_msgs::msg::LaserScan();

        laser_scan_msg.header.frame_id = "base_link";
        laser_scan_msg.header.stamp = this->now();
        laser_scan_msg.angle_min = -M_PI/2;  // Minimum angle of the laser scan
        laser_scan_msg.angle_max = M_PI/2;   // Maximum angle of the laser scan
        laser_scan_msg.angle_increment = 0.01;  // Angle increment between each measurement
        laser_scan_msg.time_increment = 0.0;  // Time between measurements 
        laser_scan_msg.scan_time = 0.033;       // Time it takes to complete one scan
        laser_scan_msg.range_min = 0.0;       // Minimum range value
        laser_scan_msg.range_max = 10.0;      // Maximum range value

        laser_scan_msg.ranges.resize(cloud->width);
        std::fill(laser_scan_msg.ranges.begin(), laser_scan_msg.ranges.end(), std::numeric_limits<double>::infinity());

        sensor_msgs::PointCloud2Iterator<float> iter_x(*cloud, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_y(*cloud, "y");
        sensor_msgs::PointCloud2Iterator<float> iter_z(*cloud, "z");

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

        scan_pub_->publish(laser_scan_msg);
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PointCloudToLaserScan>());
    rclcpp::shutdown();
    return 0;
}
