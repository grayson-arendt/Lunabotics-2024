#include <string>
#include <utility>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/clock.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp" 
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "localization_pkg/ParticleFilter.h"

/*
Author: Grayson Arendt

This program subscribes to lidar1 odometry, lidar2 odometry, and the cmd_vel topic in order to estimate
robot pose. It will only update the position if it is moving (to avoid stationary drift) and uses
lidar1 odometry as a basis, but lidar2 odometry as a secondary influence on robot pose.
*/


enum class FilterState {
    INIT,
    PREDICT
};

class StateEstimation : public rclcpp::Node {
public:

    StateEstimation() : Node("state_estimation"), state(FilterState::INIT){

        // Set parameters for std deviation x, y and yaw
        std::vector<double> deviation = {0.02, 0.03, 0.02};

        // Create 50 particles
        pf = std::make_unique<ParticleFilter>(50, deviation);
       
        lidar1_odometry_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "odom_lidar1", rclcpp::QoS(10).reliable(),
            std::bind(&StateEstimation::lidar1_odometry_callback, this, std::placeholders::_1)
        );

        lidar2_odometry_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "odom_lidar2", rclcpp::QoS(10).reliable(),
            std::bind(&StateEstimation::lidar2_odometry_callback, this, std::placeholders::_1)
        );

        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", rclcpp::QoS(10).reliable(),
            std::bind(&StateEstimation::cmd_vel_callback, this, std::placeholders::_1)
        );

        odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("filtered_odom", 10);

        // Initial pose
        x_positions.push_back(0.0);
        y_positions.push_back(0.0);
        yaws.push_back(0.0);
    }

private:

    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr cmd_vel) {
        if (cmd_vel->linear.x != 0.0 || cmd_vel->angular.z != 0.0) {
            is_moving = true;
        }
        else {
            is_moving = false;
        }
    }

    void lidar2_odometry_callback(const nav_msgs::msg::Odometry::SharedPtr odometry) {
       
        lidar2_position_x = odometry->pose.pose.position.x;
        lidar2_position_y = odometry->pose.pose.position.y;
        lidar2_orientation_x = odometry->pose.pose.orientation.x;
        lidar2_orientation_y = odometry->pose.pose.orientation.y;
        lidar2_orientation_z = odometry->pose.pose.orientation.z;
        lidar2_orientation_w = odometry->pose.pose.orientation.w;

        // Convert lidar2 orientation to yaw 
        tf2::Quaternion lidar2_q(lidar2_orientation_x, lidar2_orientation_y, lidar2_orientation_z, lidar2_orientation_w);
        lidar2_q.normalize();

        tf2::Matrix3x3 lidar2_euler(lidar2_q);
        lidar2_euler.getRPY(lidar2_roll, lidar2_pitch, lidar2_yaw);

    }

    void lidar1_odometry_callback(const nav_msgs::msg::Odometry::SharedPtr odometry) {
        if (state == FilterState::INIT) {
        
            pf->init(0, 0, 0);
            iteration = 0;

            state = FilterState::PREDICT;
        }

        else {
  
            // Predict using odometry values 
            lidar1_position_x = odometry->pose.pose.position.x;
            lidar1_position_y = odometry->pose.pose.position.y;
            lidar1_orientation_x = odometry->pose.pose.orientation.x;
            lidar1_orientation_y = odometry->pose.pose.orientation.y;
            lidar1_orientation_z = odometry->pose.pose.orientation.z;
            lidar1_orientation_w = odometry->pose.pose.orientation.w;

            // Convert lidar1 orientation to yaw
            tf2::Quaternion lidar1_q(lidar1_orientation_x, lidar1_orientation_y, lidar1_orientation_z, lidar1_orientation_w);
            lidar1_q.normalize();

            tf2::Matrix3x3 lidar1_euler(lidar1_q);
            lidar1_euler.getRPY(lidar1_roll, lidar1_pitch, lidar1_yaw);
            
            // Update odometry values with new values if moving
            if (is_moving) {
             
                // Predict particles based off lidar1 odometry, then update weights based off how close it is it lidar2 odometry
                pf->predict(lidar1_position_x, lidar1_position_y, lidar1_yaw);
                pf->updateWeight(lidar2_position_x, lidar2_position_y, lidar2_yaw);
                pf->resample();

                updated_particles = pf->getParticles();

                prime_id = pf->getPrimeParticle();

                x_positions.push_back(updated_particles[prime_id].x);
                y_positions.push_back(updated_particles[prime_id].y);
                yaws.push_back(updated_particles[prime_id].theta);
            }

            else {
                // Take last odometry value and put it back in the array
                x_positions.push_back(x_positions[iteration]);
                y_positions.push_back(y_positions[iteration]);
                yaws.push_back(yaws[iteration]);
            }

            current_x = x_positions[iteration+1];
            current_y = y_positions[iteration+1];
            current_yaw = yaws[iteration+1];

            
            //RCLCPP_INFO(this->get_logger(), "x: %f y: %f yaw: %f", current_x, current_y, current_yaw);

            // Convert back to quaternion
            current_q.setRPY(0, 0, current_yaw);

            // Publish odometry msg
            auto filtered_odom = nav_msgs::msg::Odometry();
            filtered_odom.header.stamp = this->get_clock()->now();

            filtered_odom.header.frame_id = "odom";
            filtered_odom.child_frame_id = "base_link";
            filtered_odom.pose.pose.position.x = current_x;
            filtered_odom.pose.pose.position.y = current_y;

            filtered_odom.pose.pose.orientation.x = current_q.x();
            filtered_odom.pose.pose.orientation.y = current_q.y();
            filtered_odom.pose.pose.orientation.z = current_q.z();
            filtered_odom.pose.pose.orientation.w = current_q.w();

            odom_publisher_->publish(filtered_odom);
            iteration++;
        }
    }

    FilterState state;
    tf2::Quaternion current_q;
    bool is_moving;
    int iteration;
    int prime_id;
    double current_x, current_y, current_yaw;
    double lidar1_position_x, lidar1_position_y;
    double lidar1_orientation_x, lidar1_orientation_y, lidar1_orientation_z, lidar1_orientation_w;
    double lidar1_roll, lidar1_pitch, lidar1_yaw;
    double lidar2_position_x, lidar2_position_y;
    double lidar2_orientation_x, lidar2_orientation_y, lidar2_orientation_z, lidar2_orientation_w;
    double lidar2_roll, lidar2_pitch, lidar2_yaw;
    std::vector<double> x_positions, y_positions, yaws, init_values;
    std::vector<Particle> updated_particles;
    std::unique_ptr<ParticleFilter> pf;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr lidar1_odometry_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr lidar2_odometry_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;

};

int main(int argc, char **argv) {

    rclcpp::init(argc, argv);

    auto node = std::make_shared<StateEstimation>();

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);

    while (rclcpp::ok()) {
        executor.spin();
    }

    rclcpp::shutdown();

    return 0;

}
