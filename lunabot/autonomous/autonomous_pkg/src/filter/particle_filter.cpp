#include "autonomous_pkg/ParticleFilter.h"

/**
 * @brief Constructor for ParticleFilter class.
 * @details
 * This class subscribes to lidar odometry, camera odometry, and the cmd_vel topic to estimate
 * the robot's pose. It updates the position only if the robot is moving to avoid stationary drift.
 * It uses lidar odometry as the primary source and camera odometry as a secondary influence on robot pose.
 *
 * @param particles Number of particles.
 * @param deviation Vector containing standard deviations for x, y, and theta.
 * @author Grayson Arendt
 */
ParticleFilter::ParticleFilter(int particles, std::vector<double> deviation) : Node("particle_filter"), state(FilterState::INIT)
{
    lidar_odometry_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "odom_lidar", rclcpp::QoS(10).reliable(),
        [this](const nav_msgs::msg::Odometry::SharedPtr msg)
        {
            this->lidar_odometry_callback(msg);
        });

    camera_odometry_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "rtabmap/odom", rclcpp::QoS(10).reliable(),
        [this](const nav_msgs::msg::Odometry::SharedPtr msg)
        {
            this->camera_odometry_callback(msg);
        });

    cmd_vel_subscriber_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", rclcpp::QoS(10).reliable(),
        [this](const geometry_msgs::msg::Twist::SharedPtr msg)
        {
            this->cmd_vel_callback(msg);
        });

    imu_subscriber_ = this->create_subscription<sensor_msgs::msg::Imu>(
        "imu/data", rclcpp::QoS(10).reliable(),
        [this](const sensor_msgs::msg::Imu::SharedPtr msg)
        {
            this->imu_callback(msg);
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
 * @brief Callback for the IMU subscription.
 * @param cmd_vel Received IMU message.
 */
void ParticleFilter::imu_callback(const sensor_msgs::msg::Imu::SharedPtr imu)
{
    imu_orientation_x = imu->orientation.x;
    imu_orientation_y = imu->orientation.y;
    imu_orientation_z = imu->orientation.z;
    imu_orientation_w = imu->orientation.w;

    tf2::Quaternion imu_quaternion(imu_orientation_x, imu_orientation_y, imu_orientation_z, imu_orientation_w);
    tf2::Matrix3x3 imu_euler(imu_quaternion);

    imu_euler.getRPY(imu_roll, imu_pitch, imu_yaw);
}

/**
 * @brief Callback for the camera odometry subscription.
 * @param odometry Received Odometry message.
 */
void ParticleFilter::camera_odometry_callback(const nav_msgs::msg::Odometry::SharedPtr odometry)
{
    camera_position_x = odometry->pose.pose.position.x;
    camera_position_y = odometry->pose.pose.position.y;
    camera_orientation_x = odometry->pose.pose.orientation.x;
    camera_orientation_y = odometry->pose.pose.orientation.y;
    camera_orientation_z = odometry->pose.pose.orientation.z;
    camera_orientation_w = odometry->pose.pose.orientation.w;

    // Convert camera orientation to yaw
    tf2::Quaternion camera_quaternion(camera_orientation_x, camera_orientation_y, camera_orientation_z, camera_orientation_w);
    camera_quaternion.normalize();

    tf2::Matrix3x3 camera_euler(camera_quaternion);
    camera_euler.getRPY(camera_roll, camera_pitch, camera_yaw);
}

/**
 * @brief Callback for the lidar odometry subscription.
 * @param odometry Received Odometry message.
 */
void ParticleFilter::lidar_odometry_callback(const nav_msgs::msg::Odometry::SharedPtr odometry)
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
        lidar_position_x = odometry->pose.pose.position.x;
        lidar_position_y = odometry->pose.pose.position.y;
        lidar_orientation_x = odometry->pose.pose.orientation.x;
        lidar_orientation_y = odometry->pose.pose.orientation.y;
        lidar_orientation_z = odometry->pose.pose.orientation.z;
        lidar_orientation_w = odometry->pose.pose.orientation.w;

        // Convert lidar orientation to yaw
        tf2::Quaternion lidar_quaternion(lidar_orientation_x, lidar_orientation_y, lidar_orientation_z, lidar_orientation_w);
        lidar_quaternion.normalize();

        tf2::Matrix3x3 lidar_euler(lidar_quaternion);
        lidar_euler.getRPY(lidar_roll, lidar_pitch, lidar_yaw);

        /*
        // Update odometry values with new values if moving
        if (is_moving)
        {
            // Predict particles based on lidar odometry, then update weights based on how close it is to camera odometry
            this->predict(lidar_position_x, lidar_position_y, lidar_yaw);
            this->updateWeight(camera_position_x, camera_position_y, camera_yaw);
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
        */

        this->predict(lidar_position_x, lidar_position_y, lidar_yaw);
        this->updateWeight(camera_position_x, camera_position_y, camera_yaw, imu_yaw);
        this->resample();

        updated_particles = this->getParticles();
        prime_id = this->getPrimeParticle();

        x_positions.push_back(updated_particles[prime_id].x);
        y_positions.push_back(updated_particles[prime_id].y);
        yaws.push_back(updated_particles[prime_id].theta);

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
 * @brief Creates a prediction of possible robot states via particles based off of lidar odometry.
 * @param lidar_position_x lidar x position.
 * @param lidar_position_y lidar y position.
 * @param lidar_yaw Lidar yaw orientation.
 */
void ParticleFilter::predict(double lidar_position_x, double lidar_position_y, double lidar_yaw)
{
    std::default_random_engine generator(std::chrono::system_clock::now().time_since_epoch().count());

    double std_x = std_deviation[0];
    double std_y = std_deviation[1];
    double std_yaw = std_deviation[2];

    for (int i = 0; i < num_particles; i++)
    {
        std::normal_distribution<double> dist_x(lidar_position_x, std_x);
        std::normal_distribution<double> dist_y(lidar_position_y, std_y);
        std::normal_distribution<double> dist_theta(lidar_yaw, std_yaw);

        particles[i].x = dist_x(generator);
        particles[i].y = dist_y(generator);
        particles[i].theta = dist_theta(generator);
    }
}

/**
 * @brief Calculates the weight of a particle based on lidar and camera values.
 * @param lidar_position_x Lidar x position.
 * @param lidar_position_y Lidar y position.
 * @param lidar_yaw Lidar yaw orientation.
 * @param camera_position_x Camera x position.
 * @param camera_position_y Camera y position.
 * @param camera_yaw Camera yaw orientation.
 * @param imu_yaw IMU yaw orientation.
 * @return Logarithmic weight of the particle.
 */
double ParticleFilter::calculateWeight(double lidar_position_x, double lidar_position_y, double lidar_yaw,
                                       double camera_position_x, double camera_position_y, double camera_yaw,
                                       double imu_yaw)
{
    // Calculate differences between camera and lidar positions and orientations
    double dx = camera_position_x - lidar_position_x;
    double dy = camera_position_y - lidar_position_y;
    double dtheta_camera = camera_yaw - lidar_yaw;
    double dtheta_imu = imu_yaw - lidar_yaw;

    // Calculate the components of the log weights
    double exponent_x = -(dx * dx) / (2 * std_deviation[0] * std_deviation[0]);
    double exponent_y = -(dy * dy) / (2 * std_deviation[1] * std_deviation[1]);
    double exponent_theta_camera = -(dtheta_camera * dtheta_camera) / (2 * std_deviation[2] * std_deviation[2]);
    double exponent_theta_imu = -(dtheta_imu * dtheta_imu) / (2 * std_deviation[2] * std_deviation[2]);

    // Calculate log weights for each dimension (extra weight scale for IMU, since it is more accurate angle-wise)
    double log_weight_x = exponent_x - 0.5 * log(2 * M_PI * std_deviation[0] * std_deviation[0]);
    double log_weight_y = exponent_y - 0.5 * log(2 * M_PI * std_deviation[1] * std_deviation[1]);
    double log_weight_theta_camera = exponent_theta_camera - 0.5 * log(2 * M_PI * std_deviation[2] * std_deviation[2]);
    double log_weight_theta_imu = 1.5 * exponent_theta_imu - 0.5 * log(2 * M_PI * std_deviation[2] * std_deviation[2]);

    // Sum up the log weights
    double log_weight = log_weight_x + log_weight_y + log_weight_theta_camera + log_weight_theta_imu;

    return log_weight;
}

/**
 * @brief Updates the weights of all particles based off calculated weights.
 * @param camera_position_x Camera x position.
 * @param camera_position_y Camera y position.
 * @param camera_yaw Camera yaw orientation.
 */
void ParticleFilter::updateWeight(double camera_position_x, double camera_position_y, double camera_yaw, double imu_yaw)
{
    for (int i = 0; i < num_particles; i++)
    {
        double newWeight = calculateWeight(
            particles[i].x, particles[i].y, particles[i].theta,
            camera_position_x, camera_position_y, camera_yaw, imu_yaw);

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

    std::vector<double> std_deviation = {0.02, 0.02, 0.1};

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
