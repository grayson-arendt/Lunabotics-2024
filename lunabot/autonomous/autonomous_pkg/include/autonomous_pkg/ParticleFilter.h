#ifndef PARTICLE_FILTER_H
#define PARTICLE_FILTER_H

#pragma once

#include <random>
#include <algorithm>
#include <iostream>
#include <utility>
#include <vector>
#include <cmath>
#include <chrono>
#include <limits>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/clock.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

/**
 * @struct Particle
 * @brief Represents a particle in the particle filter with id, position, orientation, and weight.
 */
struct Particle
{
    int id;
    double x;
    double y;
    double theta;
    double weight;
};

/**
 * @enum FilterState
 * @brief Enumerates the possible states of the particle filter.
 */
enum class FilterState
{
    INIT,
    PREDICT
};

/**
 * @brief Class for sensor fusion using a particle filter.
 * 
 * @author Grayson Arendt
 */
class ParticleFilter : public rclcpp::Node
{
public:
    /**
     * @brief Default constructor.
     */
    ParticleFilter();

    /**
     * @brief Parameterized constructor.
     * @param particles Number of particles in the filter.
     * @param deviation Vector of standard deviations for initializing particles.
     */
    ParticleFilter(int particles, std::vector<double> deviation);

    /**
     * @brief Callback function for lidar odometry messages.
     * @param odometry Lidar odometry message.
     */
    void lidar_odometry_callback(const nav_msgs::msg::Odometry::SharedPtr odometry);

    /**
     * @brief Callback function for camera odometry messages.
     * @param odometry Camera odometry message.
     */
    void camera_odometry_callback(const nav_msgs::msg::Odometry::SharedPtr odometry);

    /**
     * @brief Callback function for IMU messages.
     * @param imu IMU message.
     */
    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr imu);

    /**
     * @brief Callback function for cmd_vel messages.
     * @param cmd_vel Twist message for velocity control.
     */
    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr cmd_vel);

    /**
     * @brief Initializes the particle filter with given initial pose.
     * @param initial_x Initial x position.
     * @param initial_y Initial y position.
     * @param initial_theta Initial orientation (yaw).
     */
    void initialize(double initial_x, double initial_y, double initial_theta);

    /**
     * @brief Predicts the next state of the particles based on lidar odometry.
     * @param lidar_position_x X position of lidar.
     * @param lidar_position_y Y position of lidar.
     * @param lidar_yaw Yaw orientation of lidar.
     */
    void predict(double lidar_position_x, double lidar_position_y, double lidar_yaw);

    /**
     * @brief Calculates the weight of particles based on lidar and camera data.
     * @param lidar_position_x X position of lidar.
     * @param lidar_position_y Y position of lidar.
     * @param lidar_yaw Yaw orientation of lidar.
     * @param camera_position_x X position of camera.
     * @param camera_position_y Y position of camera.
     * @param camera_yaw Yaw orientation of camera.
     * @param imu_yaw Yaw orientation from IMU.
     * @return Weight of particles.
     */
    double calculateWeight(double lidar_position_x, double lidar_position_y, double lidar_yaw,
                           double camera_position_x, double camera_position_y, double camera_yaw, double imu_yaw);

    /**
     * @brief Updates the weights of particles based on camera data.
     * @param camera_position_x x position of camera.
     * @param camera_position__y y position of camera.
     * @param camera_position_yaw Yaw orientation of camera.
     */
    void updateWeight(double camera_position_x, double camera_position__y, double camera_position_yaw, double imu_yaw);

    /**
     * @brief Resamples particles based on their weights.
     */
    void resample();

    /**
     * @brief Gets the vector of particles.
     * @return Vector of Particle structures.
     */
    std::vector<Particle> getParticles();

    /**
     * @brief Gets the index of the particle with the highest weight.
     * @return Index of the particle with the highest weight.
     */
    int getPrimeParticle() const;

private:
    int num_particles;
    double weight_sum{};
    int max_weight_index{};
    double max_weight = std::numeric_limits<double>::lowest();

    std::unique_ptr<ParticleFilter> pf;
    std::vector<Particle> particles;
    std::vector<double> std_deviation;
    std::vector<double> weights;

    FilterState state;
    tf2::Quaternion current_quaternion, lidar_quaternion, camera_quaternion, imu_quaternion;
    tf2::Matrix3x3 lidar_euler, camera_euler, imu_euler;
    bool is_moving;
    int iteration;
    int prime_id;
    double imu_orientation_x, imu_orientation_y, imu_orientation_z, imu_orientation_w;
    double imu_roll, imu_pitch, imu_yaw;
    double current_x, current_y, current_yaw;
    double lidar_position_x, lidar_position_y;
    double lidar_orientation_x, lidar_orientation_y, lidar_orientation_z, lidar_orientation_w;
    double lidar_roll, lidar_pitch, lidar_yaw;
    double camera_position_x, camera_position_y;
    double camera_orientation_x, camera_orientation_y, camera_orientation_z, camera_orientation_w;
    double camera_roll, camera_pitch, camera_yaw;
    std::vector<double> x_positions, y_positions, yaws, init_values;
    std::vector<Particle> updated_particles;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr lidar_odometry_subscriber_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr camera_odometry_subscriber_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscriber_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscriber_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_publisher_;
};

#endif // PARTICLE_FILTER_H
