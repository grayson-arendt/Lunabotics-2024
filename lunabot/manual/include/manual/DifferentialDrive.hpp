#pragma once

#include <cmath>
#include <array>
#include <algorithm>

/**
 * @brief Calculates percent output for differential drive based off joystick values.
 * @details The constructor takes in an x and y value, converts it to polar,
 * adjusts theta, and limits the percent output.
 * calculate_wheel_percentOutput calculates what the left and right wheels should output.
 *
 * @author Anthony Baran
 */
class DifferentialDrive
{
public:
    /**
     * @brief Constructor for DifferentialDrive class.
     *
     * @param X X-coordinate.
     * @param Y Y-coordinate.
     * @param R Percent output.
     */
    DifferentialDrive(double X, double Y, double R)
        : percent_output(R),
          theta(std::atan2(Y, X))
    {
        // Scale theta to the turning radius output range
        double radiusScaleFactor = 5.0;
        double abstheta = std::abs(theta);

        // Multiple cases to keep the turning radius positive and between certain values even when theta is in other quadrants
        if (abstheta > 0 && abstheta < 1.57)
        {
            turning_radius = static_cast<double>(radiusScaleFactor * abstheta);
        }
        else if (abstheta > 1.57 && abstheta < 3.14)
        {
            turning_radius = static_cast<double>(radiusScaleFactor * (abstheta - (2 * (abstheta - 1.57))));
        }

        // Limit the percent output
        percent_output = std::clamp(percent_output, 0.0, 0.5000);
    }

    /**
     * @brief Get the value of theta.
     *
     * @return Theta value.
     */
    double gettheta()
    {
        return theta;
    }

    /**
     * @brief Calculate the percent output for the left and right wheels.
     *
     * @return Array containing the percent output for the left and right wheels.
     */
    std::array<double, 2> calculate_wheel_percentOutput()
    {
        // Buffer for extreme values
        if (theta >= 0.0 && theta < 0.14)
        {
            wheelPercentOutput[0] = -(percent_output / 2.5) * (2.5 - (robot_width / 2));
            wheelPercentOutput[1] = (percent_output / 2.5) * (2.5 + (robot_width / 2));
        }
        // Turning right
        else if (theta <= 1.45 && theta >= 0.14)
        {
            wheelPercentOutput[0] = (percent_output / turning_radius) * (turning_radius - (robot_width / 2));
            wheelPercentOutput[1] = (percent_output / turning_radius) * (turning_radius + (robot_width / 2));
        }
        // Going straight
        else if (theta > 1.50 && theta < 1.64)
        {
            wheelPercentOutput[0] = percent_output;
            wheelPercentOutput[1] = percent_output;
        }
        // Turning left
        else if (theta >= 1.69 && theta <= 3.00)
        {
            wheelPercentOutput[0] = (percent_output / turning_radius) * (turning_radius + (robot_width / 2));
            wheelPercentOutput[1] = (percent_output / turning_radius) * (turning_radius - (robot_width / 2));
        }
        // Buffer for extreme values
        else if (theta > 3.00)
        {
            wheelPercentOutput[0] = (percent_output / 2.5) * (2.5 + (robot_width / 2));
            wheelPercentOutput[1] = -(percent_output / 2.5) * (2.5 - (robot_width / 2));
        }
        // Buffer for extreme values
        else if (theta < -3.00)
        {
            wheelPercentOutput[0] = (percent_output / 2.5) * (2.5 + (robot_width / 2));
            wheelPercentOutput[1] = -(percent_output / 2.5) * (2.5 - (robot_width / 2));
        }
        // Turning left while going backwards
        else if (theta >= -3.00 && theta <= -1.69)
        {
            wheelPercentOutput[0] = -(percent_output / turning_radius) * (turning_radius + (robot_width / 2));
            wheelPercentOutput[1] = -(percent_output / turning_radius) * (turning_radius - (robot_width / 2));
        }
        // Going straight backwards
        else if (theta < -1.50 && theta > -1.64)
        {
            wheelPercentOutput[0] = -percent_output;
            wheelPercentOutput[1] = -percent_output;
        }
        // Turning right while going backwards
        else if (theta >= -1.45 && theta <= -0.14)
        {
            wheelPercentOutput[0] = -(percent_output / turning_radius) * (turning_radius - (robot_width / 2));
            wheelPercentOutput[1] = -(percent_output / turning_radius) * (turning_radius + (robot_width / 2));
        }
        // Buffer for extreme values
        else if (theta > -0.14 && theta < 0)
        {
            wheelPercentOutput[0] = -(percent_output / 2.5) * (2.5 - (robot_width / 2));
            wheelPercentOutput[1] = -(percent_output / 2.5) * (2.5 + (robot_width / 2));
        }

        return wheelPercentOutput;
    }

private:
    double percent_output;
    double theta;
    double turning_radius;
    double robot_width = 2.0;
    std::array<double, 2> wheelPercentOutput; // Element 0 is right and 1 is left.
};
