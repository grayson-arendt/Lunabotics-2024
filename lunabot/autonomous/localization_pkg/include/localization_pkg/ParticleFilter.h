#include <random>
#include <algorithm>
#include <iostream>
#include <utility>
#include <vector>
#include <cmath>
#include <chrono>

struct Particle {
    int id;
    double x;
    double y;
    double theta;
    double weight;
};

class ParticleFilter {
public:
    ParticleFilter(int particles, std::vector<double> deviation) : num_particles(particles), std_deviation(std::move(deviation)) {}

    void init(double init_x, double init_y, double init_theta) {
        particles.resize(num_particles);
        weights.resize(num_particles);

        for (int i = 0; i < num_particles; i++) {
            particles[i].id = i;
            particles[i].x = init_x;
            particles[i].y = init_y;
            particles[i].theta = init_theta;
            particles[i].weight = 1.0;
        }

        initialized = true;
    }

    void predict(double set_x, double set_y, double set_theta) {
        std::default_random_engine generator(std::chrono::system_clock::now().time_since_epoch().count());

        double std_x = std_deviation[0];
        double std_y = std_deviation[1];
        double std_theta = std_deviation[2];

        for (int i = 0; i < num_particles; i++) {
            std::normal_distribution<double> dist_x(set_x, std_x);
            std::normal_distribution<double> dist_y(set_y, std_y);
            std::normal_distribution<double> dist_theta(set_theta, std_theta);

            particles[i].x = dist_x(generator);
            particles[i].y = dist_y(generator);
            particles[i].theta = dist_theta(generator);
        }
    }

    double calculateWeight(double estimated_theta, double estimated_x, double estimated_y, double camera_position_x, double camera_position_y, double camera_orientation_yaw
    ) {
        double dx = camera_position_x - estimated_x;
        double dy = camera_position_y - estimated_y;
        double dtheta = camera_orientation_yaw - estimated_theta;

        double exponent_x = -(dx * dx) / (2 * std_deviation[0] * std_deviation[0]);
        double exponent_y = -(dy * dy) / (2 * std_deviation[1] * std_deviation[1]);
        double exponent_theta = -(dtheta * dtheta) / (2 * std_deviation[2] * std_deviation[2]);

        double log_weight_x = exponent_x - 0.5 * log(2 * M_PI * std_deviation[0] * std_deviation[0]);
        double log_weight_y = exponent_y - 0.5 * log(2 * M_PI * std_deviation[1] * std_deviation[1]);
        double log_weight_theta = exponent_theta - 0.5 * log(2 * M_PI * std_deviation[2] * std_deviation[2]);

        double log_weight = log_weight_x + log_weight_y + log_weight_theta;

        return log_weight;
    }

    void updateWeight(double camera_position_x, double camera_position_y, double camera_orientation_yaw) {
        for (int i = 0; i < num_particles; i++) {
   
            double newWeight = calculateWeight(
                    particles[i].theta, particles[i].x, particles[i].y,
                    camera_position_x, camera_position_y, camera_orientation_yaw
            );

            particles[i].weight = newWeight;
            weights[i] = newWeight;
            weight_sum += newWeight;

             // Update max weight and index
            if (newWeight > max_weight) {
                max_weight = newWeight;
                max_weight_index = i;
            }
        }

        for (int i = 0; i < num_particles; i++) {
            double normalized = particles[i].weight / weight_sum;
            particles[i].weight = normalized * 100;
            weights[i] = particles[i].weight;
        }
    }

    void resample() {
        std::default_random_engine generator(std::chrono::system_clock::now().time_since_epoch().count());
        std::discrete_distribution<int> distribution(weights.begin(), weights.end());
        std::vector<Particle> resampled_particles;

        for (int i = 0; i < num_particles; i++) {
            resampled_particles.push_back(particles[distribution(generator)]);
        }

        particles = resampled_particles;
    }

    bool getInitialized() const {
        return initialized;
    }

    std::vector<Particle> getParticles() {
        return particles;
    }

    int getPrimeParticle() const {
        return max_weight_index;
    }

private:
    int num_particles;
    double weight_sum{};
    bool initialized{}; 
    int max_weight_index{};
    double max_weight = std::numeric_limits<double>::lowest();

    std::vector<Particle> particles;
    std::vector<double> std_deviation;
    std::vector<double> weights;
};
