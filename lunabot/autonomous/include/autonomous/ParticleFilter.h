#ifndef PARTICLE_FILTER_H
#define PARTICLE_FILTER_H

#pragma once

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <limits>
#include <random>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"

/**
 * @struct Particle
 * @brief Represents a particle in the particle filter with id, position, orientation, and weight.
 */
struct Particle
{
    int id;
    double x;
    double y;
    double yaw;
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
 * @brief Sensor fusion using a particle filter.
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
    ParticleFilter(int particles, double deviation);

    /**
     * @brief Callback function for lidar odometry messages.
     * @param odometry Lidar odometry message.
     */
    void lidar_odometry_callback(const nav_msgs::msg::Odometry::SharedPtr odometry);

    /**
     * @brief Callback function for T265 odometry messages.
     * @param odometry T265 odometry message.
     */
    void t265_odometry_callback(const nav_msgs::msg::Odometry::SharedPtr odometry);

    /**
     * @brief Publishes odometry.
     */
    void publish_odometry();

    /**
     * @brief Estimates pose with particle filter.
     */
    void estimate_pose();

    /**
     * @brief Initializes the particle filter with given initial pose.
     * @param initial_x Initial x position.
     * @param initial_y Initial y position.
     * @param initial_yaw Initial yaw orientation.
     */
    void initialize(double initial_x, double initial_y, double initial_yaw);

    /**
     * @brief Predicts the next state of the particles based on lidar odometry.
     * @param lidar_position_x X position of lidar.
     * @param lidar_position_y Y position of lidar.
     * @param lidar_yaw Yaw orientation of lidar.
     */
    void predict(double lidar_position_x, double lidar_position_y, double lidar_yaw);

    /**
     * @brief Calculates the weight of particles based on lidar and T265 orientations.
     * @param lidar_yaw Yaw orientation of lidar.
     * @param t265_yaw Yaw orientation of T265.
     * @return Weight of particles.
     */
    double calculateWeight(double lidar_yaw, double t265_yaw);

    /**
     * @brief Updates the weights of particles based on T265 data.
     * @param t265_yaw Yaw orientation of T265.
     */
    void updateWeight(double t265_yaw);

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
    FilterState state;
    bool odometry_lost;
    int num_particles;
    int max_weight_index;
    int prime_id;
    int iteration;
    double std_yaw;
    double weight_sum;
    double log_weight;
    double max_weight = std::numeric_limits<double>::lowest();
    double current_x, current_y, current_yaw;
    double lidar_position_x, lidar_position_y;
    double lidar_orientation_x, lidar_orientation_y, lidar_orientation_z, lidar_orientation_w;
    double lidar_roll, lidar_pitch, lidar_yaw;
    double t265_orientation_x, t265_orientation_y, t265_orientation_z, t265_orientation_w;
    double t265_roll, t265_pitch, t265_yaw;
    std::vector<Particle> particles;
    std::vector<Particle> updated_particles;
    std::vector<double> weights;
    std::vector<double> x_positions, y_positions, yaws, init_values;
    tf2::Quaternion current_quaternion, lidar_quaternion, t265_quaternion;
    tf2::Matrix3x3 lidar_euler, t265_euler;
    rclcpp::TimerBase::SharedPtr t265_odometry_timer_;
    rclcpp::TimerBase::SharedPtr odometry_publisher_timer_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr lidar_odometry_subscriber_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr t265_odometry_subscriber_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_publisher_;
};

#endif // PARTICLE_FILTER_H