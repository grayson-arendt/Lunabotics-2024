#include "autonomous_pkg/ParticleFilter.h"

/**
 * @file particle_filter.cpp
 * @brief Constructor for ParticleFilter class
 * @details 
 * This class subscribes to lidar1 odometry, lidar2 odometry, and the cmd_vel topic to estimate
 * the robot's pose. It updates the position only if the robot is moving to avoid stationary drift.
 * It uses lidar1 odometry as the primary source and lidar2 odometry as a secondary influence on robot pose.
 *
 * @param particles Number of particles.
 * @param deviation Vector containing standard deviations for x, y, and theta.
 * @author Grayson Arendt
 */
ParticleFilter::ParticleFilter(int particles, std::vector<double> deviation) : Node("particle_filter"), state(FilterState::INIT)
{
    lidar1_odometry_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "odom_lidar1", rclcpp::QoS(10).reliable(),
        [this](const nav_msgs::msg::Odometry::SharedPtr msg)
        {
            this->lidar1_odometry_callback(msg);
        });

    lidar2_odometry_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "odom_lidar2", rclcpp::QoS(10).reliable(),
        [this](const nav_msgs::msg::Odometry::SharedPtr msg)
        {
            this->lidar2_odometry_callback(msg);
        });

    cmd_vel_subscriber_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", rclcpp::QoS(10).reliable(),
        [this](const geometry_msgs::msg::Twist::SharedPtr msg)
        {
            this->cmd_vel_callback(msg);
        });

    odometry_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("filtered_odom", 10);

    // Initial pose
    x_positions.push_back(0.0);
    y_positions.push_back(0.0);
    yaws.push_back(0.0);

    std_deviation = deviation;
    num_particles = particles;
}

/**
 * @brief Callback for the cmd_vel subscription.
 * @param cmd_vel Received Twist message.
 */
void ParticleFilter::cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr cmd_vel)
{
    if (cmd_vel->linear.x != 0.0 || cmd_vel->angular.z != 0.0)
    {
        is_moving = true;
    }
    else
    {
        is_moving = false;
    }
}

/**
 * @brief Callback for the lidar2 odometry subscription.
 * @param odometry Received Odometry message.
 */
void ParticleFilter::lidar2_odometry_callback(const nav_msgs::msg::Odometry::SharedPtr odometry)
{
    lidar2_position_x = odometry->pose.pose.position.x;
    lidar2_position_y = odometry->pose.pose.position.y;
    lidar2_orientation_x = odometry->pose.pose.orientation.x;
    lidar2_orientation_y = odometry->pose.pose.orientation.y;
    lidar2_orientation_z = odometry->pose.pose.orientation.z;
    lidar2_orientation_w = odometry->pose.pose.orientation.w;

    // Convert lidar2 orientation to yaw
    tf2::Quaternion lidar2_quaternion(lidar2_orientation_x, lidar2_orientation_y, lidar2_orientation_z, lidar2_orientation_w);
    lidar2_quaternion.normalize();

    tf2::Matrix3x3 lidar2_euler(lidar2_quaternion);
    lidar2_euler.getRPY(lidar2_roll, lidar2_pitch, lidar2_yaw);
}

/**
 * @brief Callback for the lidar1 odometry subscription.
 * @param odometry Received Odometry message.
 */
void ParticleFilter::lidar1_odometry_callback(const nav_msgs::msg::Odometry::SharedPtr odometry)
{
    if (state == FilterState::INIT)
    {
        this->initialize(0, 0, 0);
        iteration = 0;
        state = FilterState::PREDICT;
    }
    else
    {
        // Predict using odometry values
        lidar1_position_x = odometry->pose.pose.position.x;
        lidar1_position_y = odometry->pose.pose.position.y;
        lidar1_orientation_x = odometry->pose.pose.orientation.x;
        lidar1_orientation_y = odometry->pose.pose.orientation.y;
        lidar1_orientation_z = odometry->pose.pose.orientation.z;
        lidar1_orientation_w = odometry->pose.pose.orientation.w;

        // Convert lidar1 orientation to yaw
        tf2::Quaternion lidar1_quaternion(lidar1_orientation_x, lidar1_orientation_y, lidar1_orientation_z, lidar1_orientation_w);
        lidar1_quaternion.normalize();

        tf2::Matrix3x3 lidar1_euler(lidar1_quaternion);
        lidar1_euler.getRPY(lidar1_roll, lidar1_pitch, lidar1_yaw);

        // Update odometry values with new values if moving
        if (is_moving)
        {
            // Predict particles based on lidar1 odometry, then update weights based on how close it is to lidar2 odometry
            this->predict(lidar1_position_x, lidar1_position_y, lidar1_yaw);
            this->updateWeight(lidar2_position_x, lidar2_position_y, lidar2_yaw);
            this->resample();

            updated_particles = this->getParticles();
            prime_id = this->getPrimeParticle();

            x_positions.push_back(updated_particles[prime_id].x);
            y_positions.push_back(updated_particles[prime_id].y);
            yaws.push_back(updated_particles[prime_id].theta);
        }
        else
        {
            // Take the last odometry value and put it back in the array, since nothing has changed while stationary
            x_positions.push_back(x_positions[iteration]);
            y_positions.push_back(y_positions[iteration]);
            yaws.push_back(yaws[iteration]);
        }
        
        current_x = x_positions[iteration + 1];
        current_y = y_positions[iteration + 1];
        current_yaw = yaws[iteration + 1];

        // Convert back to quaternion
        current_quaternion.setRPY(0, 0, current_yaw);

        // Publish odometry message
        auto filtered_odom = nav_msgs::msg::Odometry();
        filtered_odom.header.stamp = this->get_clock()->now();

        filtered_odom.header.frame_id = "odom";
        filtered_odom.child_frame_id = "base_link";
        filtered_odom.pose.pose.position.x = current_x;
        filtered_odom.pose.pose.position.y = current_y;

        filtered_odom.pose.pose.orientation.x = current_quaternion.x();
        filtered_odom.pose.pose.orientation.y = current_quaternion.y();
        filtered_odom.pose.pose.orientation.z = current_quaternion.z();
        filtered_odom.pose.pose.orientation.w = current_quaternion.w();

        odometry_publisher_->publish(filtered_odom);
        iteration++;
    }
}

/**
 * @brief Initialize the particles with the given pose.
 * @param initial_x Initial x position.
 * @param initial_y Initial y position.
 * @param initial_yaw Initial theta (yaw) orientation.
 */
void ParticleFilter::initialize(double initial_x, double initial_y, double initial_yaw)
{
    particles.resize(num_particles);
    weights.resize(num_particles);

    for (int i = 0; i < num_particles; i++)
    {
        particles[i].id = i;
        particles[i].x = initial_x;
        particles[i].y = initial_y;
        particles[i].theta = initial_yaw;
        particles[i].weight = 1.0;
    }
}

/**
 * @brief Creates a prediction of possible robot states via particles based off of lidar1 odometry.
 * @param lidar1_position_x Lidar1 x position.
 * @param lidar1_position_y Lidar1 y position.
 * @param lidar1_orientation_yaw Lidar1 yaw orientation.
 */
void ParticleFilter::predict(double lidar1_position_x, double lidar1_position_y, double lidar1_orientation_yaw)
{
    std::default_random_engine generator(std::chrono::system_clock::now().time_since_epoch().count());

    double std_x = std_deviation[0];
    double std_y = std_deviation[1];
    double std_yaw = std_deviation[2];

    for (int i = 0; i < num_particles; i++)
    {
        std::normal_distribution<double> dist_x(lidar1_position_x, std_x);
        std::normal_distribution<double> dist_y(lidar1_position_y, std_y);
        std::normal_distribution<double> dist_theta(lidar1_orientation_yaw, std_yaw);

        particles[i].x = dist_x(generator);
        particles[i].y = dist_y(generator);
        particles[i].theta = dist_theta(generator);
    }
}

/**
 * @brief Calculates the weight of a particle based on lidar1 and lidar2 values.
 * @param lidar1_position_x Lidar1 x position.
 * @param lidar1_position_y Lidar1 y position.
 * @param lidar1_orientation_yaw Lidar1 yaw orientation.
 * @param lidar2_position_x Lidar2 x position.
 * @param lidar2_position_y Lidar2 y position.
 * @param lidar2_orientation_yaw Lidar2 yaw orientation.
 * @return Logarithmic weight of the particle.
 */
double ParticleFilter::calculateWeight(double lidar1_position_x, double lidar1_position_y, double lidar1_orientation_yaw, 
                                       double lidar2_position_x, double lidar2_position_y, double lidar2_orientation_yaw)
{
    double dx = lidar2_position_x - lidar1_position_x;
    double dy = lidar2_position_y - lidar1_position_y;
    double dtheta = lidar2_orientation_yaw - lidar1_orientation_yaw;

    double exponent_x = -(dx * dx) / (2 * std_deviation[0] * std_deviation[0]);
    double exponent_y = -(dy * dy) / (2 * std_deviation[1] * std_deviation[1]);
    double exponent_theta = -(dtheta * dtheta) / (2 * std_deviation[2] * std_deviation[2]);

    double log_weight_x = exponent_x - 0.5 * log(2 * M_PI * std_deviation[0] * std_deviation[0]);
    double log_weight_y = exponent_y - 0.5 * log(2 * M_PI * std_deviation[1] * std_deviation[1]);
    double log_weight_theta = exponent_theta - 0.5 * log(2 * M_PI * std_deviation[2] * std_deviation[2]);

    double log_weight = log_weight_x + log_weight_y + log_weight_theta;

    return log_weight;
}

/**
 * @brief Updates the weights of all particles based off calculated weights.
 * @param lidar2_position_x Lidar2 x position.
 * @param lidar2_position_y Lidar2 y position.
 * @param lidar2_orientation_yaw Lidar2 yaw orientation.
 */
void ParticleFilter::updateWeight(double lidar2_position_x, double lidar2_position_y, double lidar2_orientation_yaw)
{
    for (int i = 0; i < num_particles; i++)
    {
        double newWeight = calculateWeight(
            particles[i].theta, particles[i].x, particles[i].y,
            lidar2_position_x, lidar2_position_y, lidar2_orientation_yaw);

        particles[i].weight = newWeight;
        weights[i] = newWeight;
        weight_sum += newWeight;

        // Update max weight and index
        if (newWeight > max_weight)
        {
            max_weight = newWeight;
            max_weight_index = i;
        }
    }

    for (int i = 0; i < num_particles; i++)
    {
        double normalized = particles[i].weight / weight_sum;
        particles[i].weight = normalized * 100;
        weights[i] = particles[i].weight;
    }
}

/**
 * @brief Resamples particles based on their weights.
 */
void ParticleFilter::resample()
{
    std::default_random_engine generator(std::chrono::system_clock::now().time_since_epoch().count());
    std::discrete_distribution<int> distribution(weights.begin(), weights.end());
    std::vector<Particle> resampled_particles;

    for (int i = 0; i < num_particles; i++)
    {
        resampled_particles.push_back(particles[distribution(generator)]);
    }

    particles = resampled_particles;
}

/**
 * @brief Retrieves the current particles.
 * @return Vector of Particle objects representing the particles.
 */
std::vector<Particle> ParticleFilter::getParticles()
{
    return particles;
}

/**
 * @brief Retrieves the index of the particle with the highest weight (prime particle).
 * @return Index of the prime particle.
 */
int ParticleFilter::getPrimeParticle() const
{
    return max_weight_index;
}

/**
 * @brief Main function.
 * 
 * Initializes and spins the ParticleFilter node.
 */
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    std::vector<double> std_deviation = {0.001, 0.001, 0.001};

    auto particleFilter = std::make_shared<ParticleFilter>(500, std_deviation);

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(particleFilter);

    while (rclcpp::ok())
    {
        executor.spin();
    }

    rclcpp::shutdown();

    return 0;
}
