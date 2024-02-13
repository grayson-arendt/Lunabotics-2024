#pragma once

#include <array>
#include <chrono>
#include <cmath>
#include <string>
#include <vector>

#include "rclcpp/logging.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/string.hpp"

#include <cv_bridge/cv_bridge.h>
#include <opencv2/aruco.hpp>
#include <opencv2/aruco/dictionary.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>

/**
 * @brief Detects AprilTags and estimates pose.
 *
 * @author Grayson Arendt
 */
class AprilTag
{
  public:
    /**
     * @brief Default constructor.
     *
     * @param image Input image.
     * @param size Tag size.
     * @param id Tag ID.
     */
    AprilTag(const sensor_msgs::msg::Image &image, const double size, const int id);

    /**
     * @brief Detect AprilTags in the input image and estimate their pose.
     */
    void detectTag();

    /**
     * @brief Set the detected flag.
     *
     * @param value Flag value.
     */
    void setDetected(bool value);

    /**
     * @brief Get the detected flag.
     *
     * @return Flag indicating whether a tag is detected.
     */
    bool getDetected();

  private:
    /**
     * @brief Calculate the distance from the camera to the detected tag.
     *
     * @param rvec Rotation vector.
     * @param tvec Translation vector.
     * @return The distance.
     */
    double calculateDistance(cv::Vec3d &rvec, cv::Vec3d &tvec);

    int tagId;
    double tagSize;
    double x_coord;
    double y_coord;
    double z_coord;
    sensor_msgs::msg::Image inputImage;
    bool isDetected;
};

// Method implementations

AprilTag::AprilTag(const sensor_msgs::msg::Image &image, const double size, const int id)
    : tagId(id), tagSize(size), inputImage(image), isDetected(false)
{
}

void AprilTag::detectTag()
{
    setDetected(false);

    cv_bridge::CvImagePtr image_ptr = cv_bridge::toCvCopy(inputImage, inputImage.encoding);

    std::vector<int> markerIds;
    std::vector<cv::Vec3d> rvecs, tvecs;
    std::vector<std::vector<cv::Point2f>> markerCorners;

    cv::Mat cameraMatrix =
        (cv::Mat1d(3, 3) << 278.5971299187092, 0, 323.0268692272516, 0, 284.902753361877, 235.7951612448387, 0, 0, 1);
    cv::Mat distortionCoefficients = (cv::Mat1d(1, 5) << -0.08455656319490816, 0.09495737180245305,
                                      0.005608739170439007, -0.007920839212581778, -0.05865648378565153);

    cv::Ptr<cv::aruco::DetectorParameters> parameters = cv::aruco::DetectorParameters::create();
    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_APRILTAG_36h11);

    // Detect tag and estimate pose
    cv::aruco::detectMarkers(image_ptr->image, dictionary, markerCorners, markerIds, parameters);
    cv::aruco::estimatePoseSingleMarkers(markerCorners, tagSize, cameraMatrix, distortionCoefficients, rvecs, tvecs);

    for (std::vector<int>::size_type i = 0; i < markerIds.size(); i++)
    {

        cv::aruco::drawDetectedMarkers(image_ptr->image, markerCorners, markerIds);
        cv::aruco::drawAxis(image_ptr->image, cameraMatrix, distortionCoefficients, rvecs[i], tvecs[i], 0.1);

        if (markerIds[i] == tagId)
        {

            setDetected(true);
            z_coord = calculateDistance(rvecs[i], tvecs[i]);
        }
    }

    /* For debugging
    cv::imshow("Detected Markers", image_ptr->image);
    cv::waitKey(0); // Wait for a key event to close the window
     */
}

void AprilTag::setDetected(bool value)
{
    isDetected = value;
}

bool AprilTag::getDetected()
{
    return isDetected;
}

double AprilTag::calculateDistance(cv::Vec3d &rvec, cv::Vec3d &tvec)
{

    double distance;

    cv::Mat r_mat;

    // Convert vector to matrix format (for multiplication)
    cv::Mat t_mat = cv::Mat(tvec).reshape(1);

    // Convert 3D rotation vector to matrix using rodrigues function
    cv::Rodrigues(rvec, r_mat);

    // Inverse both rotation matrix and translation matrix
    cv::Mat inversePose = r_mat.inv() * -t_mat;

    // Get z value from matrix, (x, y, z) format
    distance = inversePose.at<double>(2, 0);

    return distance;
}
