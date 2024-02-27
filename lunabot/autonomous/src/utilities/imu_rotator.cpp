#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include <string>

/**
 * @brief Rotates linear acceleration and angular velocity in an IMU message for both D455 and T265 camersa.
 * @details The D455/T265 orientation is non-standard and does not work correctly with external packages.
 *
 * @author Grayson Arendt
 */
class IMURotator : public rclcpp::Node
{
  public:
    /**
     * @brief Constructor for IMURotator.
     */
    IMURotator() : Node("imu_rotator")
    {

        d455_subscriber_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "d455/imu", 10, std::bind(&IMURotator::d455_callback, this, std::placeholders::_1));
        t265_subscriber_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "t265/imu", 10, std::bind(&IMURotator::t265_callback, this, std::placeholders::_1));

        d455_publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("d455/imu_enu", 10);
        t265_publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("t265/imu_enu", 10);
    }

  private:
    /**
     * @brief Callback function for processing and publishing rotated D455 IMU messages.
     *
     * @param msg The received IMU message.
     */
    void d455_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        // Copy the received message
        auto rotated_msg = std::make_shared<sensor_msgs::msg::Imu>(*msg);

        // Transform linear acceleration
        rotated_msg->linear_acceleration.y = msg->linear_acceleration.x;
        rotated_msg->linear_acceleration.x = msg->linear_acceleration.y;
        rotated_msg->linear_acceleration.z = -msg->linear_acceleration.z;

        // Transform angular velocity
        rotated_msg->angular_velocity.y = msg->angular_velocity.x;
        rotated_msg->angular_velocity.x = msg->angular_velocity.y;
        rotated_msg->angular_velocity.z = -msg->angular_velocity.z;

        // Publish the transformed message
        d455_publisher_->publish(*rotated_msg);
    }

    /**
     * @brief Callback function for processing and publishing rotated T265 IMU messages.
     *
     * @param msg The received IMU message.
     */
    void t265_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        // Copy the received message
        auto rotated_msg = std::make_shared<sensor_msgs::msg::Imu>(*msg);

        // Transform linear acceleration
        rotated_msg->linear_acceleration.y = -msg->linear_acceleration.x;
        rotated_msg->linear_acceleration.x = -msg->linear_acceleration.y;
        rotated_msg->linear_acceleration.z = msg->linear_acceleration.z;

        // Transform angular velocity
        rotated_msg->angular_velocity.y = -msg->angular_velocity.x;
        rotated_msg->angular_velocity.x = -msg->angular_velocity.y;
        rotated_msg->angular_velocity.z = msg->angular_velocity.z;

        // Publish the transformed message
        t265_publisher_->publish(*rotated_msg);
    }
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr d455_subscriber_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr t265_subscriber_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr d455_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr t265_publisher_;
};

/**
 * @brief Main function.
 *
 * Initializes and spins the IMURotator node.
 */
int main(int argc, char *argv[])
{

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<IMURotator>());
    rclcpp::shutdown();
    return 0;
}
