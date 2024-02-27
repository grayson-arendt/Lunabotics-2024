#include "autonomous/ParticleFilter.h"

/**
 * @brief Constructor for ParticleFilter class.
 * @details
 * This class subscribes to lidar odometry, T265 odometry, and IMU data to
 * estimate the robot's pose. It uses lidar odometry as the primary source,
 * T265 odometry as a secondary influence on robot pose, and IMU as a third
 * influence.

 * @param particles Number of particles.
 * @param deviation Vector containing standard deviations for x, y, and yaw.
 * @author Grayson Arendt
 */
ParticleFilter::ParticleFilter(int particles, std::vector<double> deviation)
    : Node("particle_filter"), state(FilterState::INIT)
{
    lidar_odometry_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "odom_lidar", rclcpp::QoS(10).reliable(),
        [this](const nav_msgs::msg::Odometry::SharedPtr msg) { this->lidar_odometry_callback(msg); });

    t265_odometry_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "t265/pose/sample", rclcpp::QoS(10).reliable(),
        [this](const nav_msgs::msg::Odometry::SharedPtr msg) { this->t265_odometry_callback(msg); });

    imu_subscriber_ = this->create_subscription<sensor_msgs::msg::Imu>(
        "imu/data", rclcpp::QoS(10).reliable(),
        [this](const sensor_msgs::msg::Imu::SharedPtr msg) { this->imu_callback(msg); });

    odometry_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("odometry/filtered", 10);

    odometry_publisher_timer_ =
        create_wall_timer(std::chrono::milliseconds(100), [this]() { this->publish_odometry(); });
    
    std_deviation = deviation;
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
    t265_position_x = odometry->pose.pose.position.x;
    t265_position_y = odometry->pose.pose.position.y;
    t265_orientation_x = odometry->pose.pose.orientation.x;
    t265_orientation_y = odometry->pose.pose.orientation.y;
    t265_orientation_z = odometry->pose.pose.orientation.z;
    t265_orientation_w = odometry->pose.pose.orientation.w;

    // Convert t265 orientation to yaw
    tf2::Quaternion t265_quaternion(t265_orientation_x, t265_orientation_y, t265_orientation_z,
                                      t265_orientation_w);
    t265_quaternion.normalize();

    tf2::Matrix3x3 t265_euler(t265_quaternion);
    t265_euler.getRPY(t265_roll, t265_pitch, t265_yaw);
}

/**
 * @brief Callback for the IMU subscription.
 * @param imu Received IMU message.
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
        state = FilterState::PREDICT;
    }
    else
    {
        this->predict(lidar_position_x, lidar_position_y, lidar_yaw);
        this->updateWeight(t265_position_x, t265_position_y, t265_yaw, imu_yaw);
        this->resample();

        updated_particles = this->getParticles();
        prime_id = this->getPrimeParticle();
        
        current_x = updated_particles[prime_id].x;
        current_y =updated_particles[prime_id].y;
        current_yaw = updated_particles[prime_id].yaw;

        current_quaternion.setRPY(0, 0, current_yaw);
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

    double std_x = std_deviation[0];
    double std_y = std_deviation[1];
    double std_yaw = std_deviation[2];

    for (int i = 0; i < num_particles; i++)
    {
        std::normal_distribution<double> dist_x(lidar_position_x, std_x);
        std::normal_distribution<double> dist_y(lidar_position_y, std_y);
        std::normal_distribution<double> dist_yaw(lidar_yaw, std_yaw);

        particles[i].x = dist_x(generator);
        particles[i].y = dist_y(generator);
        particles[i].yaw = dist_yaw(generator);
    }
}

/**
 * @brief Calculates the weight of a particle based on lidar, T265, and IMU
 * values.
 * @param lidar_position_x Lidar x position.
 * @param lidar_position_y Lidar y position.
 * @param lidar_yaw Lidar yaw orientation.
 * @param t265_position_x T265 x position.
 * @param t265_position_y T265 y position.
 * @param t265_yaw T265 yaw orientation.
 * @param imu_yaw IMU yaw orientation.
 * @return Logarithmic weight of the particle.
 */
double ParticleFilter::calculateWeight(double lidar_position_x, double lidar_position_y, double lidar_yaw,
                                       double t265_position_x, double t265_position_y, double t265_yaw,
                                       double imu_yaw)
{
    // Calculate differences between t265 and lidar positions and orientations
    double dx = t265_position_x - lidar_position_x;
    double dy = t265_position_y - lidar_position_y;
    double dyaw_t265 = t265_yaw - lidar_yaw;
    double dyaw_imu = imu_yaw - lidar_yaw;

    // Calculate the components of the log weights
    double exponent_x = -(dx * dx) / (2 * std_deviation[0] * std_deviation[0]);
    double exponent_y = -(dy * dy) / (2 * std_deviation[1] * std_deviation[1]);
    double exponent_yaw_t265 = -(dyaw_t265 * dyaw_t265) / (2 * std_deviation[2] * std_deviation[2]);
    double exponent_yaw_imu = -(dyaw_imu * dyaw_imu) / (2 * std_deviation[2] * std_deviation[2]);

    // Calculate log weights for each dimension (extra weight scale for IMU,
    // since it is more accurate angle-wise)
    double log_weight_x = exponent_x - 0.5 * log(2 * M_PI * std_deviation[0] * std_deviation[0]);
    double log_weight_y = exponent_y - 0.5 * log(2 * M_PI * std_deviation[1] * std_deviation[1]);
    double log_weight_yaw_t265 = exponent_yaw_t265 - 0.5 * log(2 * M_PI * std_deviation[2] * std_deviation[2]);
    double log_weight_yaw_imu = 1.5 * exponent_yaw_imu - 0.5 * log(2 * M_PI * std_deviation[2] * std_deviation[2]);

    // temporarily removed imu weight
    log_weight = log_weight_x + log_weight_y + log_weight_yaw_t265;
    
    return log_weight;
}

/**
 * @brief Updates the weights of all particles based off calculated weights.
 * @param t265_position_x T265 x position.
 * @param t265_position_y T265 y position.
 * @param t265_yaw T265 yaw orientation.
 * @param imu_yaw IMU yaw orientation.
 */
void ParticleFilter::updateWeight(double t265_position_x, double t265_position_y, double t265_yaw, double imu_yaw)
{
    for (int i = 0; i < num_particles; i++)
    {
        double newWeight = calculateWeight(particles[i].x, particles[i].y, particles[i].yaw, t265_position_x,
                                           t265_position_y, t265_yaw, imu_yaw);

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

    std::vector<double> std_deviation = {0.01, 0.01, 0.01};

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