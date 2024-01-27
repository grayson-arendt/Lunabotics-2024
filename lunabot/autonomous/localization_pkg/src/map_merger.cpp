#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

class OccupancyGridMergerNode : public rclcpp::Node
{
public:
  OccupancyGridMergerNode() : Node("occupancy_grid_merger_node")
  {
    // Create publishers and subscribers
    occupancy_grid1_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      "rtabmap/map", 10, std::bind(&OccupancyGridMergerNode::occupancyGrid1Callback, this, std::placeholders::_1));

    occupancy_grid2_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      "slam_toolbox/map", 10, std::bind(&OccupancyGridMergerNode::occupancyGrid2Callback, this, std::placeholders::_1));

    merged_occupancy_grid_pub_ =
      create_publisher<nav_msgs::msg::OccupancyGrid>("map", 10);

    // Initialize the merged occupancy grid
    merged_occupancy_grid_.info.width = 0;
    merged_occupancy_grid_.info.height = 0;
  }

private:
  void occupancyGrid1Callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
  {
    RCLCPP_INFO(get_logger(), "Received occupancy_grid1 message with timestamp %f", msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9);

    // Resize merged occupancy grid if necessary
    resizeMergedOccupancyGrid(msg->info.width, msg->info.height);

    // Copy values from occupancy_grid1
    for (size_t i = 0; i < merged_occupancy_grid_.data.size(); ++i)
    {
      merged_occupancy_grid_.data[i] = msg->data[i];
    }

    // Set timestamp for merged occupancy grid
    merged_occupancy_grid_.header.stamp = rclcpp::Clock().now();

    // Publish the merged occupancy grid
    merged_occupancy_grid_pub_->publish(merged_occupancy_grid_);
  }

  void occupancyGrid2Callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
  {
    RCLCPP_INFO(get_logger(), "Received occupancy_grid2 message with timestamp %f", msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9);

    // Resize merged occupancy grid if necessary
    resizeMergedOccupancyGrid(msg->info.width, msg->info.height);

    // Combine values from occupancy_grid2 (prioritize occupancy_grid2 over occupancy_grid1)
    for (size_t i = 0; i < merged_occupancy_grid_.data.size(); ++i)
    {
      if (msg->data[i] != std::numeric_limits<int8_t>::min() && msg->data[i] != std::numeric_limits<int8_t>::max())
      {
        // Prioritize occupancy_grid2 over occupancy_grid1
        merged_occupancy_grid_.data[i] = msg->data[i];
      }
    }

    // Set timestamp for merged occupancy grid
    merged_occupancy_grid_.header.stamp = rclcpp::Clock().now();

    // Publish the merged occupancy grid
    merged_occupancy_grid_pub_->publish(merged_occupancy_grid_);
  }

  void resizeMergedOccupancyGrid(unsigned int width, unsigned int height)
  {
    // Resize merged occupancy grid if necessary
    if (width > merged_occupancy_grid_.info.width || height > merged_occupancy_grid_.info.height)
    {
      merged_occupancy_grid_.info.width = std::max(width, merged_occupancy_grid_.info.width);
      merged_occupancy_grid_.info.height = std::max(height, merged_occupancy_grid_.info.height);
      merged_occupancy_grid_.data.resize(merged_occupancy_grid_.info.width * merged_occupancy_grid_.info.height, 0);
    }
  }

  // Subscribers and Publishers
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr occupancy_grid1_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr occupancy_grid2_sub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr merged_occupancy_grid_pub_;

  // Storage for merged occupancy grid
  nav_msgs::msg::OccupancyGrid merged_occupancy_grid_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OccupancyGridMergerNode>());
  rclcpp::shutdown();
}