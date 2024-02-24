#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"

/**
 * @brief Rotates linear acceleration and angular velocity in an IMU message for T265 camera.
 * @details The T265 orientation is non-standard and does not work correctly with external packages.
 *
 * @author Grayson Arendt
 */
class IMURotatorT265 : public rclcpp::Node
{
  public:
    /**
     * @brief Constructor for IMURotatorT265.
     */
    IMURotatorT265() : Node("imu_rotator_t265")
    {

        subscriber_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "t265/imu", 10, std::bind(&IMURotatorT265::imu_callback, this, std::placeholders::_1));

        publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("t265/imu_enu", 10);
    }

  private:
    /**
     * @brief Callback function for processing and publishing rotated IMU messages.
     *
     * @param msg The received IMU message.
     */
    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
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
        publisher_->publish(*rotated_msg);
    }
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subscriber_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher_;
};

/**
 * @brief Main function.
 *
 * Initializes and spins the IMURotatorT265 node.
 */
int main(int argc, char *argv[])
{

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<IMURotatorT265>());
    rclcpp::shutdown();
    return 0;
}
