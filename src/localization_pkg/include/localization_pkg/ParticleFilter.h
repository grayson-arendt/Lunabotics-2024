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

    /*
    Init_x, init_y, and init_theta (RAD) come from pose estimation from initial apriltag
    We care about the yaw (z-axis rotation) since this is 2D
    init_theta = atan2(r_mat[1,0] - r_mat[0,0]);
    init_x = t_mat[0];
    init_y = t_mat[1];
     */

    void init(double init_x, double init_y, double init_theta) {

        particles.resize(num_particles);
        weights.resize(num_particles);

        for (int i = 0; i < num_particles; i++){
            particles[i].id = i;
            particles[i].x = init_x;
            particles[i].y = init_y;
            particles[i].theta = init_theta;
            particles[i].weight = 1.0;
        }

        initialized = true;
    }

    void predict(double set_x, double set_y, double set_theta) {

        // Random number generator
        std::default_random_engine generator(std::chrono::system_clock::now().time_since_epoch().count());

        double std_x = std_deviation[0];
        double std_y = std_deviation[1];
        double std_theta = std_deviation[2];

        for (int i = 0; i < num_particles; i++) {

            /* Prediction is what we set x, y, and theta to be */

            // Adding Gaussian noise to positions/angle based off of standard deviation (of encoder in this case)
            std::normal_distribution<double> dist_x(set_x,std_x);
            std::normal_distribution<double> dist_y(set_y,std_y);
            std::normal_distribution<double> dist_theta(set_theta,std_theta);

            // Picking random point on distribution to set to a particle
            particles[i].x = dist_x(generator);
            particles[i].y = dist_y(generator);
            particles[i].theta = dist_theta(generator);
        }
    }

    // Calculate weight via a univariate distribution formula (variables do not inherently rely on each other)
    double calculateWeight(double estimated_theta, double estimated_x, double estimated_y, double camera_theta, double camera_x, double camera_y) {

        // Calculate squared differences
        double dx = camera_x - estimated_x;
        double dy = camera_y - estimated_y;
        double dtheta = camera_theta - estimated_theta;

        // Calculate the exponents for each dimension separately
        double exponent_x = -(dx * dx) / (2 * std_deviation[0] * std_deviation[0]);
        double exponent_y = -(dy * dy) / (2 * std_deviation[1] * std_deviation[1]);
        double exponent_theta = -(dtheta * dtheta) / (2 * std_deviation[2] * std_deviation[2]);

        // Take the log to avoid underflow errors
        double log_weight_x = exponent_x - 0.5 * log(2 * M_PI * std_deviation[0] * std_deviation[0]);
        double log_weight_y = exponent_y - 0.5 * log(2 * M_PI * std_deviation[1] * std_deviation[1]);
        double log_weight_theta = exponent_theta - 0.5 * log(2 * M_PI * std_deviation[2] * std_deviation[2]);

        // Sum all weights
        double log_weight = log_weight_x + log_weight_y + log_weight_theta;

        //std::cout << log_weight << std::endl;

        return log_weight;
    }


    void updateWeight(bool bermTagDetected, double camera_x, double camera_y, double camera_theta) {

        // Add weights to particles
        for (int i = 0; i < num_particles; i++) {

            if (bermTagDetected) {
                double newWeight = calculateWeight(particles[i].theta, particles[i].x, particles[i].y, camera_theta, camera_x, camera_y);

                particles[i].weight = newWeight;
                weights[i] = newWeight;
                weight_sum += newWeight;
            }

            else {
                particles[i].weight = 0;
                weights[i] = 0;
            }
        }

        // Normalize points after all the weights are calculated
        for (int i = 0; i < num_particles; i++) {

            double normalized = particles[i].weight / weight_sum;
            particles[i].weight = normalized * 100; //
            weights[i] = particles[i].weight;
        }
    }

    void resample() {

        // Randomly select a particle and kill it based off weight (large weights survive)

        // Random number generator
        std::default_random_engine generator(std::chrono::system_clock::now().time_since_epoch().count());

        // Produces random integers with probability based on weight of int/sum of weights
        std::discrete_distribution<int> distribution(weights.begin(), weights.end());
        std::vector<Particle> resampled_particles;

        // Add each surviving particle to the array
        for (int i = 0; i < num_particles; i++) {
            resampled_particles.push_back(particles[distribution(generator)]);
        }

        // Particles vector consists of only the survivors now
        particles = resampled_particles;
    }

    bool getInitialized() const {
        return initialized;
    }

    std::vector<Particle> getParticles() {
        return particles;
    }

private:
    int num_particles;
    double weight_sum{};
    bool initialized{};

    std::vector<Particle> particles;
    std::vector<double> std_deviation;
    std::vector<double> weights;
};