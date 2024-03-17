#include "autonomous/ParticleFilter.h"

/**
 * @brief Constructor for ParticleFilter class.
 * @details
 * This class subscribes to lidar odometry and T265 odometry estimate the robot's pose.
 * It uses lidar odometry as the primary source and then uses the T265's yaw as it is
 * very accurate while turning, but not accurate while driving forward and backwards.
 *
 * @param particles Number of particles.
 * @param deviation Vector containing standard deviations for x, y, and yaw.
 * @author Grayson Arendt
 */
ParticleFilter::ParticleFilter(int particles, double deviation) : Node("particle_filter"), state(FilterState::INIT)
{
    lidar_odometry_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "odom_lidar", rclcpp::QoS(10).reliable(),
        [this](const nav_msgs::msg::Odometry::SharedPtr msg) { this->lidar_odometry_callback(msg); });

    t265_odometry_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "t265/pose/sample", rclcpp::QoS(10).reliable(),
        [this](const nav_msgs::msg::Odometry::SharedPtr msg) { this->t265_odometry_callback(msg); });

    odometry_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("filtered_odom", 10);

    odometry_publisher_timer_ =
        create_wall_timer(std::chrono::milliseconds(100), [this]() { this->publish_odometry(); });

    // Initial pose
    x_positions.push_back(0.0);
    y_positions.push_back(0.0);
    yaws.push_back(0.0);

    std_yaw = deviation;
    num_particles = particles;
}

/**
 * @brief Callback for the lidar odometry subscription.
 * @param odometry Received odometry message.
 */
void ParticleFilter::lidar_odometry_callback(const nav_msgs::msg::Odometry::SharedPtr odometry)
{
    lidar_position_x = odometry->pose.pose.position.x;
    lidar_position_y = odometry->pose.pose.position.y;
    lidar_orientation_x = odometry->pose.pose.orientation.x;
    lidar_orientation_y = odometry->pose.pose.orientation.y;
    lidar_orientation_z = odometry->pose.pose.orientation.z;
    lidar_orientation_w = odometry->pose.pose.orientation.w;

    // Convert lidar orientation to yaw
    tf2::Quaternion lidar_quaternion(lidar_orientation_x, lidar_orientation_y, lidar_orientation_z,
                                     lidar_orientation_w);
    lidar_quaternion.normalize();

    tf2::Matrix3x3 lidar_euler(lidar_quaternion);
    lidar_euler.getRPY(lidar_roll, lidar_pitch, lidar_yaw);
}

/**
 * @brief Callback for the t265 odometry subscription.
 * @param odometry Received odometry message.
 */
void ParticleFilter::t265_odometry_callback(const nav_msgs::msg::Odometry::SharedPtr odometry)
{
    t265_orientation_x = odometry->pose.pose.orientation.x;
    t265_orientation_y = odometry->pose.pose.orientation.y;
    t265_orientation_z = odometry->pose.pose.orientation.z;
    t265_orientation_w = odometry->pose.pose.orientation.w;

    // Convert t265 orientation to yaw
    tf2::Quaternion t265_quaternion(t265_orientation_x, t265_orientation_y, t265_orientation_z, t265_orientation_w);
    t265_quaternion.normalize();

    tf2::Matrix3x3 t265_euler(t265_quaternion);
    t265_euler.getRPY(t265_roll, t265_pitch, t265_yaw);
}

/**
 * @brief Publishes odometry.
 */
void ParticleFilter::publish_odometry()
{
    this->estimate_pose();

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
}

/**
 * @brief Estimates pose with particle filter.
 */
void ParticleFilter::estimate_pose()
{
    if (state == FilterState::INIT)
    {
        this->initialize(0, 0, 0);
        iteration = 0;
        state = FilterState::PREDICT;
    }
    else
    {
        this->predict(lidar_position_x, lidar_position_y, lidar_yaw);
        this->updateWeight(t265_yaw);
        this->resample();

        updated_particles = this->getParticles();
        prime_id = this->getPrimeParticle();

        x_positions.push_back(updated_particles[prime_id].x);
        y_positions.push_back(updated_particles[prime_id].y);
        yaws.push_back(updated_particles[prime_id].yaw);

        current_x = x_positions[iteration + 1];
        current_y = y_positions[iteration + 1];
        current_yaw = yaws[iteration + 1];

        current_quaternion.setRPY(0, 0, current_yaw);

        iteration++;
    }
}

/**
 * @brief Initialize the particles with the given pose.
 * @param initial_x Initial x position.
 * @param initial_y Initial y position.
 * @param initial_yaw Initial yaw orientation.
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
        particles[i].yaw = initial_yaw;
        particles[i].weight = 1.0;
    }
}

/**
 * @brief Creates a prediction of possible robot states via particles based off
 * of lidar odometry.
 * @param lidar_position_x Lidar x position.
 * @param lidar_position_y Lidar y position.
 * @param lidar_yaw Lidar yaw orientation.
 */
void ParticleFilter::predict(double lidar_position_x, double lidar_position_y, double lidar_yaw)
{
    std::default_random_engine generator(std::chrono::system_clock::now().time_since_epoch().count());

    for (int i = 0; i < num_particles; i++)
    {
        std::normal_distribution<double> dist_yaw(lidar_yaw, std_yaw);

        particles[i].x = lidar_position_x;
        particles[i].y = lidar_position_y;
        particles[i].yaw = dist_yaw(generator);
    }
}

/**
 * @brief Calculates the weight of a particle based on lidar and T265 pose values.
 * @param lidar_position_x Lidar x position.
 * @param lidar_position_y Lidar y position.
 * @param lidar_yaw Lidar yaw orientation.
 * @param t265_yaw T265 yaw orientation.
 * @return Logarithmic weight of the particle.
 */
double ParticleFilter::calculateWeight(double lidar_yaw, double t265_yaw)
{
    // Calculate differences between T265 and lidar orientations
    double dyaw = t265_yaw - lidar_yaw;
    double exponent_yaw = -(dyaw * dyaw) / (2 * std_yaw * std_yaw);
    double log_weight_yaw = exponent_yaw - 0.5 * log(2 * M_PI * std_yaw * std_yaw);

    return log_weight_yaw;
}

/**
 * @brief Updates the weights of all particles based off calculated weights.
 * @param t265_yaw t265 yaw orientation.
 */
void ParticleFilter::updateWeight(double t265_yaw)
{
    for (int i = 0; i < num_particles; i++)
    {
        double newWeight = calculateWeight(particles[i].yaw, t265_yaw);

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
 * @brief Retrieves the index of the particle with the highest weight (prime
 * particle).
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

    double std_yaw = 0.005;

    auto particleFilter = std::make_shared<ParticleFilter>(500, std_yaw);

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(particleFilter);

    while (rclcpp::ok())
    {
        executor.spin();
    }

    rclcpp::shutdown();

    return 0;
}