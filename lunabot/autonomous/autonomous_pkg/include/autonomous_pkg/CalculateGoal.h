#ifndef CALCULATE_GOAL_H
#define CALCULATE_GOAL_H

#include <vector>
#include <cmath>
#include <array>

/**
 * @brief Calculates the goal position based on AprilTag distances.
 *
 * @details This class calculates the goal position based on the distance from two AprilTags.
 * It outputs a vector pointing from the robot to the goal.
 *
 * @author Zayd Khan
 */
class CalculateGoal
{
public:
    /**
     * @brief Calculate the possible robot positions based on AprilTag distances.
     *
     * @param code1_pos Position of the first AprilTag.
     * @param code2_pos Position of the second AprilTag.
     * @param d1 Distance from the robot to the first AprilTag.
     * @param d2 Distance from the robot to the second AprilTag.
     * @param d3 Distance between the two AprilTags.
     * @return An array containing two possible robot positions.
     */
    std::array<std::vector<float>, 2> robot_position(std::vector<float> code1_pos, std::vector<float> code2_pos, float d1, float d2, float d3)
    {
        // dx_12 and dy_12 are the horizontal and vertical displacement from
        // QR code 1 to QR code 2.
        float dx_12 = code2_pos[0] - code1_pos[0];
        float dy_12 = code2_pos[1] - code1_pos[1];

        // using law of cosines to calculate cos_2.
        // note: I write d1^2 - d2^2 as (d1 + d2) * (d1 - d2) to cut out
        // 1 use of multiplication
        float cos_2 = (d1 + d2) * (d1 - d2) + d3 * d3;
        cos_2 /= (2 * d1 * d3);

        // using pythagorean trig identity to calculate sin_2
        float sin_2 = sqrt(1 - cos_2 * cos_2);

        // r is a vector of length d1 pointing from QR code 1 to the robot.
        // one component of r lies parallel to the vector pointing from
        // QR code 1 to QR code 2, and one lies perpendicular.
        // these are the magnitudes of those components.
        float r_parallel_magnitude = cos_2 * d1;
        float r_perp_magnitude = sin_2 * d1;

        // we convert these components into vectors using the previously
        // calculated magnitudes and the appropriate unit vectors.
        float r_parallel[2] = {r_parallel_magnitude * dx_12 / d3, r_parallel_magnitude * dy_12 / d3};
        float r_perp[2] = {r_perp_magnitude * dy_12 / d3, -1 * r_perp_magnitude * dx_12 / d3};

        // the two possible positions of the robot
        std::vector<float> robot_pos_1{r_parallel[0] + r_perp[0] + code1_pos[0], r_parallel[1] + r_perp[1] + code1_pos[1]};
        std::vector<float> robot_pos_2{r_parallel[0] - r_perp[0] + code1_pos[0], r_parallel[1] - r_perp[1] + code1_pos[1]};

        // returned as an array of float vectors
        std::array<std::vector<float>, 2> robot_positions = {robot_pos_1, robot_pos_2};
        return robot_positions;
    }

    /**
     * @brief Calculate the goal vector pointing from the robot to the goal.
     *
     * @param robot_position Position of the robot.
     * @param goal_position Position of the goal.
     * @return The goal vector.
     */
    std::vector<float> calculate_goal(std::vector<float> robot_position, std::vector<float> goal_position)
    {
        std::vector<float> goal_vector{goal_position[0] - robot_position[0], goal_position[1] - robot_position[1]};
        return goal_vector;
    }
};

#endif