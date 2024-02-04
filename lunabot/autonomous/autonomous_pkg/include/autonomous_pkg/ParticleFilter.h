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
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/clock.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

struct Particle
{
    int id;
    double x;
    double y;
    double theta;
    double weight;
};

enum class FilterState
{
    INIT,
    PREDICT
};

class ParticleFilter : public rclcpp::Node
{
public:
    ParticleFilter();
    ParticleFilter(int particles, std::vector<double> deviation);

    void lidar1_odometry_callback(const nav_msgs::msg::Odometry::SharedPtr odometry);
    void lidar2_odometry_callback(const nav_msgs::msg::Odometry::SharedPtr odometry);
    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr cmd_vel);

    void initialize(double initial_x, double initial_y, double initial_theta);
    void predict(double lidar1_position_x, double lidar1_position_y, double lidar1_orientation_yaw);
    double calculateWeight(double lidar1_position_x, double lidar1_position_y, double lidar1_orientation_yaw, 
                            double lidar2_position_x, double lidar2_position_y, double lidar2_orientation_yaw);
    void updateWeight(double lidar2_position_x, double lidar2_position__y, double lidar2_position_yaw);
    void resample();
    
    std::vector<Particle> getParticles();
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
