#include "localization_pkg/ParticleFilter.h"
#include "localization_pkg/AprilTag.h"

#include <string>
#include <utility>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/clock.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <opencv2/core/mat.hpp>

std::vector<Particle> updated_particles;

enum class FilterState {
    INIT,
    PREDICT
};

class StateEstimation : public rclcpp::Node {
public:

    StateEstimation() : Node("positionEstimation"), state(FilterState::INIT){

        std::vector<double> deviation = {0.02, 0.03, 0.02, 0.05};
        pf = std::make_unique<ParticleFilter>(5, deviation);
        movement_values = {0, 0, 0};  // Values for robot movement

        // Create subscriber (lambda expression)
        imageSub_ = this->create_subscription<sensor_msgs::msg::Image>("camera/color/image_raw", rclcpp::QoS(10).reliable(),
        [this](const sensor_msgs::msg::Image::SharedPtr image) {
            if (state == FilterState::INIT) {
                // Perform initialization logic 
                 if (!initTagsDetected) {
                    getInitPosition(image, 0.10, 0, 1, 0.30, 0.30);
                } else {
                    init_values = getInitValues();
                    pf->init(init_values[0], init_values[1], init_values[2]);
                    movement_values = simulateMovement(init_values[0], init_values[1], init_values[2]);
                    state = FilterState::PREDICT;
                }
            } else if (state == FilterState::PREDICT) {
                // Perform prediction logic 
                pf->predict(movement_values[0], movement_values[1], movement_values[2]);
                pf->updateWeight(true, 5.0, 4.0, 3.0);
                pf->resample();
                updated_particles = pf->getParticles();

                for (auto &&particle : updated_particles) {
                    readParticle(particle);
                }
                
                movement_values = simulateMovement(movement_values[0],movement_values[1],movement_values[2]);
            }

            resetTimer();
        });

        // Set up a timer to check for liveliness
        imageTimer_ = create_wall_timer(std::chrono::seconds(1),
        [this]() {
            RCLCPP_WARN(get_logger(), "\033[0;36mCAMERA:\033[0m \033[1;31mDISCONNECTED\033[0m");
        });
    }

private:

    void resetTimer() {
        RCLCPP_WARN(this->get_logger(), "\033[0;36mCAMERA:\033[0m \033[1;32mOK\033[0m");
        imageTimer_->reset();
    }

    void getInitPosition(const sensor_msgs::msg::Image::SharedPtr inputImage, double size, int x_id, int y_id, double x_tag_corner, double y_tag_corner) {
        AprilTag x_tag(*inputImage, size, x_id);
        AprilTag y_tag(*inputImage, size, y_id);

        x_tag.detectTag();
        y_tag.detectTag();

        // Keep trying to detect tags until they're both detected
        if ((!x_tag.getDetected() || !y_tag.getDetected()) || (x_tag.z_coord <= 0.0 || y_tag.z_coord <= 0.0)) {
            RCLCPP_INFO(this->get_logger(), "\033[0;35mATTEMPTING TO DETECT BOTH TAGS...\033[0m ");
        }

        else {

            initTagsDetected = true;
            /* Getting only x values from tags, since x is the value to left or right. Tags are on perpendicular walls, so
            they can represent both x and y-axis just from each separate x value */

            double camera_x = y_tag.z_coord;
            double camera_y = x_tag.z_coord;

            RCLCPP_INFO_ONCE(this->get_logger(), "\033[0;34mINITIAL POSITION: (%f, %f) (meters)\033[0m", camera_x, camera_y);
            double camera_theta = (M_PI / 2) - atan2(-camera_y, -camera_x);

            RCLCPP_INFO_ONCE(this->get_logger(), "\033[0;34mINITIAL ANGLE: %f (radians)\033[0m", camera_theta);

            std::vector<double> values = {camera_x, camera_y, camera_theta};
            setInitValues(values);

            RCLCPP_INFO_ONCE(this->get_logger(), "\033[0;32mINITIALIZATION: SUCCESS\033[0m");

        }
    }

    // Simulate rotation and driving
    std::vector<double> simulateMovement(double init_x, double init_y, double init_theta) {
        bool clockwise;
        std::string direction;

        // Generate random number in range [0, 1]
        std::random_device device;
        std::mt19937 generator(device());
        std::uniform_real_distribution<> distance_distribution(0, 1);
        std::uniform_real_distribution<> angle_distribution(0, 2*M_PI);

        double random_dist = distance_distribution(generator);
        double random_theta = angle_distribution(generator);

        if (fmod((random_theta - init_theta + 2*M_PI), 2*M_PI) < M_PI){
            direction = "CLOCKWISE";

            /* Rotate robot here (left wheel forward, right wheel backward)
             *
             *
             *
             *
             */
        }

        else {
            direction = "COUNTER-CLOCKWISE";
            
            /* Rotate robot here (right wheel forward, left wheel backward)
           *
           *
           *
           *
           */

        }

        RCLCPP_INFO(this->get_logger(), "ANGLE: %f (radians) DIRECTION: %s", random_theta, direction.c_str());

        /* Drive robot here
         *
         *
         *
         *
         */

        RCLCPP_INFO(this->get_logger(), "DISTANCE: %fm", random_dist);

        // Calculate new x and y robot coords:
        double new_x = init_x + (random_dist * cos(random_theta));
        double new_y = init_y + (random_dist * sin(random_theta));

        RCLCPP_INFO(this->get_logger(), "POSITION: (%f,%f) (meters)", new_x, new_y);

        std::vector<double> new_values = {new_x, new_y, random_theta};
        return new_values;
    }

    void readParticle(Particle particle) {
        RCLCPP_INFO(this->get_logger(), "PARTICLE %i: (x: %f, y: %f, θ: %f, weight: %f)", particle.id, particle.x, particle.y, particle.theta, particle.weight);
    }

    void setInitValues(std::vector<double> values) {
        init_values = std::move(values);
    }

    std::vector<double> getInitValues() {
        return init_values;
    }

    FilterState state;
    bool initTagsDetected{};
    std::vector<double> init_values;
    std::vector<double> movement_values;
    std::unique_ptr<ParticleFilter> pf;
    rclcpp::TimerBase::SharedPtr imageTimer_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr imageSub_;
};

int main(int argc, char **argv) {
    // Start ROS2
    rclcpp::init(argc, argv);

    auto node = std::make_shared<StateEstimation>();

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);

    while (rclcpp::ok()) {
        executor.spin();
    }

    // Shutdown node
    rclcpp::shutdown();

    return 0;
}
