#ifndef CALCULATE_GOAL_H
#define CALCULATE_GOAL_H
#include <vector>
#include <math.h>
#include <array>
using namespace std;

// author: Zayd Khan
// This is the header file for CalculateGoal, 
// which uses the distance from two april tags in
// order to accurately locate the robot, then output
// a vector pointing from the robot to the goal.
class CalculateGoal {
    public:
        array<vector<float>, 2> robot_position(vector<float> code1_pos, vector<float> code2_pos, float d1, float d2, float d3) {
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
            vector<float> robot_pos_1 {r_parallel[0] + r_perp[0] + code1_pos[0], r_parallel[1] + r_perp[1] + code1_pos[1]};
            vector<float> robot_pos_2 {r_parallel[0] - r_perp[0] + code1_pos[0], r_parallel[1] - r_perp[1] + code1_pos[1]};
            
            // returned as an array of float vectors
            array<vector<float>, 2> robot_positions = {robot_pos_1, robot_pos_2};
            return robot_positions;
        }
    
        // calculates a vector pointing from the robot to the goal, given robot position and goal position
        vector<float> calculate_goal(vector<float> robot_position, vector<float> goal_position) {
            vector<float> goal_vector {goal_position[0] - robot_position[0], goal_position[1] - robot_position[1]};
        }
};

#endif 