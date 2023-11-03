#pragma once
#include <cmath>
#include <array>
#include <algorithm>


class DifferentialDrive
{
public:
    DifferentialDrive(double X, double Z)
    : percent_output(X),
      theta(Z)
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

        // limit the percent output
        percent_output = std::clamp(percent_output, 0.0, 0.3000);
    }
    
    double gettheta()
    {
        return theta;
    }

    std::array<double, 2> calculate_wheel_percentOutput()
    {
        // Buffer for extreme values
        if (theta >= 0 && theta < 0.14)
        {
            wheelPercentOutput[0] = -(percent_output / 2.5) * (2.5 - (robot_width / 2));
            wheelPercentOutput[1] = (percent_output / 2.5) * (2.5 + (robot_width / 2));
        }
        // turning right
        else if (theta <= 1.45 && theta >= 0.14)
        {
            wheelPercentOutput[0] = (percent_output / turning_radius) * (turning_radius - (robot_width / 2));
            wheelPercentOutput[1] = (percent_output / turning_radius) * (turning_radius + (robot_width / 2));
        }
        // going straight
        else if (theta > 1.45 && theta < 1.69)
        {
            wheelPercentOutput[0] = percent_output;
            wheelPercentOutput[1] = percent_output;
        }
        // turning left
        else if (theta >= 1.69 && theta <= 3.00)
        {
            wheelPercentOutput[0] = (percent_output / turning_radius) * (turning_radius + (robot_width / 2));
            wheelPercentOutput[1] = (percent_output / turning_radius) * (turning_radius - (robot_width / 2));
        }
        // buffer for extreme values
        else if (theta > 3.00)
        {
            wheelPercentOutput[0] = (percent_output / 2.5) * (2.5 + (robot_width / 2));
            wheelPercentOutput[1] = -(percent_output / 2.5) * (2.5 - (robot_width / 2));
        }
        // buffer for extreme values
        else if (theta < -3.00)
        {
            wheelPercentOutput[0] = (percent_output / 2.5) * (2.5 + (robot_width / 2));
            wheelPercentOutput[1] = -(percent_output / 2.5) * (2.5 - (robot_width / 2));            
        }
        // turning left while going backwards
        else if (theta >= -3.00 && theta <= -1.69)
        {
            wheelPercentOutput[0] = -(percent_output / turning_radius) * (turning_radius + (robot_width / 2));
            wheelPercentOutput[1] = -(percent_output / turning_radius) * (turning_radius - (robot_width / 2));
        }
        // going straight backwards
        else if (theta < -1.45 && theta > -1.69)
        {
            wheelPercentOutput[0] = -percent_output;
            wheelPercentOutput[1] = -percent_output;
        }
        // turning right while going backwards
        else if (theta >= -1.45 && theta <= -0.14)
        {
            wheelPercentOutput[0] = -(percent_output / turning_radius) * (turning_radius - (robot_width / 2));
            wheelPercentOutput[1] = -(percent_output / turning_radius) * (turning_radius + (robot_width / 2));
        }
        // buffer for extreme values
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
    double robot_width = 0.5;
    std::array<double, 2> wheelPercentOutput; // Element 0 is right and 1 is left
};