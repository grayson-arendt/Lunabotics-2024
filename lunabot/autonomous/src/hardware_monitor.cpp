#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

class HardwareMonitor : public rclcpp::Node
{
public:
    HardwareMonitor() : Node("hardware_monitor")
    {
        d455_subscriber_ = this->create_subscription<sensor_msgs::msg::Image>(
            "d455/color/image_raw", rclcpp::QoS(10).reliable(),
            [this](const sensor_msgs::msg::Image::SharedPtr)
            {
                this->resetTimer(d455_timer_, d455_valid);
            });

        t265_subscriber_ = this->create_subscription<sensor_msgs::msg::Image>(
            "t265/fisheye1/image_raw", rclcpp::QoS(10).reliable(),
            [this](const sensor_msgs::msg::Image::SharedPtr)
            {
                this->resetTimer(t265_timer_, t265_valid);
            });

        lidar1_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "scan", rclcpp::QoS(10).reliable(),
            [this](const sensor_msgs::msg::LaserScan::SharedPtr)
            {
                this->resetTimer(lidar1_timer_, lidar1_valid);
            });

        lidar2_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "scan2", rclcpp::QoS(10).reliable(),
            [this](const sensor_msgs::msg::LaserScan::SharedPtr)
            {
                this->resetTimer(lidar2_timer_, lidar2_valid);
            });

        d455_timer_ = create_wall_timer(std::chrono::seconds(5), [this]()
                                        { this->checkConnection(d455_timer_, d455_valid, "D455"); });

        t265_timer_ = create_wall_timer(std::chrono::seconds(5), [this]()
                                        { this->checkConnection(t265_timer_, t265_valid, "T265"); });

        lidar1_timer_ = create_wall_timer(std::chrono::seconds(5), [this]()
                                          { this->checkConnection(lidar1_timer_, lidar1_valid, "LIDAR S2L"); });

        lidar2_timer_ = create_wall_timer(std::chrono::seconds(5), [this]()
                                          { this->checkConnection(lidar2_timer_, lidar2_valid, "LIDAR A3"); });

        all_sensors_timer_ = create_wall_timer(std::chrono::seconds(1), [this]()
                                               { if (d455_valid && t265_valid && lidar1_valid && lidar2_valid) {
            RCLCPP_INFO(get_logger(), "\033[0;36mSENSOR CONDITION:\033[0m \033[1;32mGOOD\033[0m");
        } });

        d455_valid = false;
        t265_valid = false;
        lidar1_valid = false;
        lidar2_valid = false;
    }

    void resetTimer(rclcpp::TimerBase::SharedPtr timer, bool &valid_flag)
    {
        timer->reset();
        valid_flag = true;
    }

    // Function to check connection status
    void checkConnection(rclcpp::TimerBase::SharedPtr timer, bool &valid_flag, const std::string &sensor_name)
    {
        if (!valid_flag)
        {
            RCLCPP_WARN(get_logger(), "\033[0;36m%s:\033[0m \033[1;31mNO CONNECTION, CHECK CABLE\033[0m", sensor_name.c_str());
        }
        valid_flag = false;
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr d455_subscriber_, t265_subscriber_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar1_subscriber_, lidar2_subscriber_;
    rclcpp::TimerBase::SharedPtr d455_timer_, t265_timer_, lidar1_timer_, lidar2_timer_, all_sensors_timer_;
    bool d455_valid, t265_valid, lidar1_valid, lidar2_valid;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<HardwareMonitor>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
