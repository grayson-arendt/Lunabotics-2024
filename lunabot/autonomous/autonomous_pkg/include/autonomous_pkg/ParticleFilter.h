#ifndef PARTICLE_FILTER_H
#define PARTICLE_FILTER_H

/**
 * @file ParticleFilter.h
 * @brief ParticleFilter class definition for implementing a particle filter localization algorithm.
 */

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
 * @class ParticleFilter
 * @brief Implements a particle filter for localization using sensor data.
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
     * @brief Callback function for lidar1 odometry messages.
     * @param odometry Lidar1 odometry message.
     */
    void lidar1_odometry_callback(const nav_msgs::msg::Odometry::SharedPtr odometry);

    /**
     * @brief Callback function for lidar2 odometry messages.
     * @param odometry Lidar2 odometry message.
     */
    void lidar2_odometry_callback(const nav_msgs::msg::Odometry::SharedPtr odometry);

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
     * @brief Predicts the next state of the particles based on lidar1 odometry.
     * @param lidar1_position_x X position of lidar1.
     * @param lidar1_position_y Y position of lidar1.
     * @param lidar1_orientation_yaw Yaw orientation of lidar1.
     */
    void predict(double lidar1_position_x, double lidar1_position_y, double lidar1_orientation_yaw);

    /**
     * @brief Calculates the weight of particles based on lidar1 and lidar2 data.
     * @param lidar1_position_x X position of lidar1.
     * @param lidar1_position_y Y position of lidar1.
     * @param lidar1_orientation_yaw Yaw orientation of lidar1.
     * @param lidar2_position_x X position of lidar2.
     * @param lidar2_position_y Y position of lidar2.
     * @param lidar2_orientation_yaw Yaw orientation of lidar2.
     * @return Weight of particles.
     */
    double calculateWeight(double lidar1_position_x, double lidar1_position_y, double lidar1_orientation_yaw,
                            double lidar2_position_x, double lidar2_position_y, double lidar2_orientation_yaw);

    /**
     * @brief Updates the weights of particles based on lidar2 data.
     * @param lidar2_position_x x position of lidar2.
     * @param lidar2_position__y y position of lidar2.
     * @param lidar2_position_yaw Yaw orientation of lidar2.
     */
    void updateWeight(double lidar2_position_x, double lidar2_position__y, double lidar2_position_yaw);

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
    tf2::Quaternion current_quaternion, lidar1_quaternion, lidar2_quaternion;
    tf2::Matrix3x3 lidar1_euler, lidar2_euler;
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
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr lidar1_odometry_subscriber_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr lidar2_odometry_subscriber_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscriber_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_publisher_;

};

#endif // PARTICLE_FILTER_H
