#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include <cmath>
#include <vector>

class PointCloudToLaserScanConverter : public rclcpp::Node
{
public:
    PointCloudToLaserScanConverter() : Node("pointcloud_to_laserscan_converter")
    {
        pointcloud_subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "rtabmap_d455/scan_cloud", 10, std::bind(&PointCloudToLaserScanConverter::pointcloud_callback, this, std::placeholders::_1));
        laserscan_publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan>("merged_scan", 10);
    }

private:
    void pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr pointcloud_msg)
    {
        // Extract parameters from pointcloud message
        float angle_min = -M_PI / 2;
        float angle_max = M_PI / 2;
        float angle_increment = M_PI / (pointcloud_msg->width - 1);
        float time_increment = 0.0;
        float scan_time = 0.033;
        float range_min = 0.0;
        float range_max = 10.0;

        // Fill ranges array with range values from pointcloud
        size_t num_points = pointcloud_msg->width;
        std::vector<float> ranges(num_points, std::numeric_limits<float>::infinity());

        sensor_msgs::PointCloud2ConstIterator<float> iter_x(*pointcloud_msg, "x");
        sensor_msgs::PointCloud2ConstIterator<float> iter_y(*pointcloud_msg, "y");
        sensor_msgs::PointCloud2ConstIterator<float> iter_z(*pointcloud_msg, "z");

        for (size_t i = 0; i < num_points; ++i, ++iter_x, ++iter_y, ++iter_z)
        {
            float x = *iter_x;
            float y = *iter_y;
            float z = *iter_z;

            float range = std::sqrt(x * x + y * y + z * z); // Euclidean distance
            float angle = std::atan2(y, x);                 // Calculate the angle

            if (angle >= angle_min && angle <= angle_max)
            {
                size_t index = static_cast<size_t>((angle - angle_min) / angle_increment);
                if (index < num_points)
                {
                    ranges[index] = std::min(ranges[index], range);
                }
            }
        }

        // Create LaserScan message
        sensor_msgs::msg::LaserScan laserscan_msg;
        laserscan_msg.header = pointcloud_msg->header;
        laserscan_msg.angle_min = angle_min;
        laserscan_msg.angle_max = angle_max;
        laserscan_msg.angle_increment = angle_increment;
        laserscan_msg.time_increment = time_increment;
        laserscan_msg.scan_time = scan_time;
        laserscan_msg.range_min = range_min;
        laserscan_msg.range_max = range_max;
        laserscan_msg.ranges = ranges;

        // Publish LaserScan message
        laserscan_publisher_->publish(laserscan_msg);
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_subscriber_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr laserscan_publisher_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PointCloudToLaserScanConverter>());
    rclcpp::shutdown();
    return 0;
}
