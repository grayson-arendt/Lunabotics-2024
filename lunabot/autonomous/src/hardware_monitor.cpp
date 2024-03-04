#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
/**
 * @brief Node for for monitoring hardware topics to check liveliness.
 *
 * @author Grayson Arendt
 */

class HardwareMonitor : public rclcpp::Node
{
  public:
    /**
     * @brief Constructor for HardwareMonitor class
     */
    HardwareMonitor() : Node("hardware_monitor")
    {

        d455_subscriber_ = this->create_subscription<sensor_msgs::msg::Image>(
            "d455/color/image_raw", rclcpp::QoS(10).reliable(),
            [this](const sensor_msgs::msg::Image::SharedPtr msg) {this->d455_timer_->reset();});

        t265_subscriber_ = this->create_subscription<sensor_msgs::msg::Image>(
            "t265/fisheye1/image_raw", rclcpp::QoS(10).reliable(),
            [this](const sensor_msgs::msg::Image::SharedPtr msg) {this->t265_timer_->reset();});

        lidar1_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "scan", rclcpp::QoS(10).reliable(),
            [this](const sensor_msgs::msg::LaserScan::SharedPtr msg) {this->lidar1_timer_->reset();});

        lidar2_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "scan2", rclcpp::QoS(10).reliable(),
            [this](const sensor_msgs::msg::LaserScan::SharedPtr msg) {this->lidar2_timer_->reset();});

        d455_timer_ = create_wall_timer(std::chrono::seconds(1), [this]() {
            RCLCPP_WARN(get_logger(), "\033[0;36mD455:\033[0m \033[1;31mNO CONNECTION\033[0m");
        });

        t265_timer_ = create_wall_timer(std::chrono::seconds(1), [this]() {
            RCLCPP_WARN(get_logger(), "\033[0;36mT265:\033[0m \033[1;31mNO CONNECTION\033[0m");
        });

        lidar1_timer_ = create_wall_timer(std::chrono::seconds(1), [this]() {
            RCLCPP_WARN(get_logger(), "\033[0;36mLIDAR S2L:\033[0m \033[1;31mNO CONNECTION\033[0m");
        });

        lidar2_timer_ = create_wall_timer(std::chrono::seconds(1), [this]() {
            RCLCPP_WARN(get_logger(), "\033[0;36mLIDAR A3:\033[0m \033[1;31mNO CONNECTION\033[0m");
        });
    }

  private:
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr d455_subscriber_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr t265_subscriber_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar1_subscriber_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar2_subscriber_;
    rclcpp::TimerBase::SharedPtr d455_timer_;
    rclcpp::TimerBase::SharedPtr t265_timer_;
    rclcpp::TimerBase::SharedPtr lidar1_timer_;
    rclcpp::TimerBase::SharedPtr lidar2_timer_;
};

/**
 * @brief Main function.
 *
 * Initializes and spins the HardwareMonitor node
 */
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<HardwareMonitor>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
