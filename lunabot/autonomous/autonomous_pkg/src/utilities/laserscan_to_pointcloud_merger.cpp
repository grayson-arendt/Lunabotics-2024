#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl_conversions/pcl_conversions.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"

/**
 * @brief Converts two LaserScan messages into one PointCloud2 message.
 *
 * @details This class subscribes to two LaserScan topics, converts each LaserScan message
 * to a PCL PointCloud, merges the PointClouds, and publishes the merged PointCloud
 * as a PointCloud2 message.
 *
 * @author Grayson Arendt
 */
class LaserScanToPointCloudMerger : public rclcpp::Node
{
  public:
    /**
     * @brief Constructor for the LaserScanToPointCloudMerger class.
     */
    LaserScanToPointCloudMerger() : Node("laserscan_to_pointcloud_merger")
    {
        // Subscribe to LaserScan topics
        scan1_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "scan1", 10, std::bind(&LaserScanToPointCloudMerger::scan1_callback, this, std::placeholders::_1));
        scan2_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "scan2", 10, std::bind(&LaserScanToPointCloudMerger::scan2_callback, this, std::placeholders::_1));

        // Create publisher for PointCloud2 messages
        cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("rtabmap_d455/scan_cloud", 10);

        // Create timer to publish merged PointCloud periodically
        timer_ = this->create_wall_timer(std::chrono::milliseconds(100),
                                         std::bind(&LaserScanToPointCloudMerger::publish_merged_pointcloud, this));
    }

  private:
    /**
     * @brief Callback function for processing messages from scan1 topic.
     *
     * @param msg The received LaserScan message.
     */
    void scan1_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        cloud1_ = convert_scan_to_pointcloud(msg, 0.0, 0.0, 0.0);
    }

    /**
     * @brief Callback function for processing messages from scan2 topic.
     *
     * @param msg The received LaserScan message.
     */
    void scan2_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        cloud2_ = convert_scan_to_pointcloud(msg, M_PI, 0.74835, 0.0);
    }

    /**
     * @brief Converts LaserScan message to PCL PointCloud.
     *
     * @param msg The received LaserScan message.
     * @return The converted PCL PointCloud.
     */
    pcl::PointCloud<pcl::PointXYZ>::Ptr convert_scan_to_pointcloud(const sensor_msgs::msg::LaserScan::SharedPtr msg,
                                                                   float angle_offset, float translation_offset_x,
                                                                   float translation_offset_y)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

        float angle_min = msg->angle_min;
        float angle_increment = msg->angle_increment;

        // Convert polar coordinates to cartesian coordinates and apply transformation
        for (size_t i = 0; i < msg->ranges.size(); ++i)
        {
            float angle = angle_min + i * angle_increment + angle_offset;
            // Ensure angle is within [-pi, pi] range
            while (angle > M_PI)
                angle -= 2 * M_PI;
            while (angle < -M_PI)
                angle += 2 * M_PI;

            float x = msg->ranges[i] * std::cos(angle) + translation_offset_x;
            float y = msg->ranges[i] * std::sin(angle) + translation_offset_y;
            cloud->push_back(pcl::PointXYZ(x, y, 0.0));
        }

        return cloud;
    }

    /**
     * @brief Publishes the merged PointCloud as a PointCloud2 message.
     */
    void publish_merged_pointcloud()
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr merged_cloud_(new pcl::PointCloud<pcl::PointXYZ>);

        auto cloud_out = sensor_msgs::msg::PointCloud2();

        pcl::concatenate(*cloud1_, *cloud2_, *merged_cloud_);
        pcl::toROSMsg(*merged_cloud_, cloud_out);
      
        cloud_out.header.frame_id = "lidar1_link";
        cloud_pub_->publish(cloud_out);
    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan1_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan2_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LaserScanToPointCloudMerger>());
    rclcpp::shutdown();
    return 0;
}
