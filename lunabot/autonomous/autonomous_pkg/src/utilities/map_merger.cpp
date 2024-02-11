#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

/**
 * @brief Class for merging two occupancy grid maps.
 * 
 * @author Grayson Arendt
 */
class MapMerger : public rclcpp::Node
{
public:
  /**
   * @brief Constructor for MapMerger.
   */
  MapMerger() : Node("occupancy_grid_merger_node")
  {
    occupancy_grid1_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
        "rtabmap/map", 10, std::bind(&MapMerger::occupancyGrid1Callback, this, std::placeholders::_1));

    occupancy_grid2_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
        "slam_toolbox/map", 10, std::bind(&MapMerger::occupancyGrid2Callback, this, std::placeholders::_1));

    merged_occupancy_grid_pub_ =
        create_publisher<nav_msgs::msg::OccupancyGrid>("map", 10);

    merged_occupancy_grid_.info.width = 0;
    merged_occupancy_grid_.info.height = 0;
  }

private:
  /**
   * @brief Callback function for processing occupancy_grid1 messages.
   *
   * @param msg The received occupancy_grid1 message.
   */
  void occupancyGrid1Callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
  {
    RCLCPP_INFO(get_logger(), "Received occupancy_grid1 message with timestamp %f", msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9);

    resizeMergedOccupancyGrid(msg->info.width, msg->info.height);

    // Copy values from occupancy_grid1
    for (size_t i = 0; i < merged_occupancy_grid_.data.size(); ++i)
    {
      merged_occupancy_grid_.data[i] = msg->data[i];
    }

    merged_occupancy_grid_.header.stamp = rclcpp::Clock().now();
    merged_occupancy_grid_pub_->publish(merged_occupancy_grid_);
  }

  /**
   * @brief Callback function for processing occupancy_grid2 messages.
   *
   * @param msg The received occupancy_grid2 message.
   */
  void occupancyGrid2Callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
  {
    RCLCPP_INFO(get_logger(), "Received occupancy_grid2 message with timestamp %f", msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9);

    resizeMergedOccupancyGrid(msg->info.width, msg->info.height);

    for (size_t i = 0; i < merged_occupancy_grid_.data.size(); ++i)
    {
      if (msg->data[i] != std::numeric_limits<int8_t>::min() && msg->data[i] != std::numeric_limits<int8_t>::max())
      {
        // Prioritize occupancy_grid2 over occupancy_grid1
        merged_occupancy_grid_.data[i] = msg->data[i];
      }
    }

    merged_occupancy_grid_.header.stamp = rclcpp::Clock().now();
    merged_occupancy_grid_pub_->publish(merged_occupancy_grid_);
  }

  /**
   * @brief Resize the merged occupancy grid if necessary.
   *
   * @param width The width of the received occupancy grid.
   * @param height The height of the received occupancy grid.
   */
  void resizeMergedOccupancyGrid(unsigned int width, unsigned int height)
  {
    if (width > merged_occupancy_grid_.info.width || height > merged_occupancy_grid_.info.height)
    {
      merged_occupancy_grid_.info.width = std::max(width, merged_occupancy_grid_.info.width);
      merged_occupancy_grid_.info.height = std::max(height, merged_occupancy_grid_.info.height);
      merged_occupancy_grid_.data.resize(merged_occupancy_grid_.info.width * merged_occupancy_grid_.info.height, 0);
    }
  }

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr occupancy_grid1_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr occupancy_grid2_sub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr merged_occupancy_grid_pub_;
  nav_msgs::msg::OccupancyGrid merged_occupancy_grid_;
};

/**
 * @brief Main function.
 *
 * Initializes and spins the MapMerger node.
 */
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapMerger>());
  rclcpp::shutdown();
  return 0;
}
