#pragma once

#include <chrono>
#include <cmath>
#include <string>
#include <vector>
#include <array>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/logging.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/string.hpp"

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/aruco/dictionary.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/mat.hpp>
#include <cv_bridge/cv_bridge.h> // Uses OpenCV 4.2

/*
Author: Grayson Arendt
This class detects an AprilTag of a certain id, estimates the pose, and outputs
the distance based off input image and tag size.
*/

class AprilTag {
public:
    int tagId;
    double tagSize;
    double x_coord;
    double y_coord;
    double z_coord;
    sensor_msgs::msg::Image inputImage;

    void detectTag() {

        setDetected(false);

        cv_bridge::CvImagePtr image_ptr = cv_bridge::toCvCopy(inputImage, inputImage.encoding);

        std::vector<int> markerIds;
        std::vector<cv::Vec3d> rvecs, tvecs;
        std::vector<std::vector<cv::Point2f>> markerCorners;

        cv::Mat cameraMatrix = (cv::Mat1d(3, 3)
                << 383.4185742519996, 0, 309.4326377845713, 0, 385.0909007102088, 240.749949733094, 0, 0, 1);
        cv::Mat distortionCoefficients = (cv::Mat1d(1, 5)
                << -0.06792929080519726, 0.08058277259698843, -0.001690544521662593, -0.0008235437909836152, -0.04417756393089296);

        cv::Ptr<cv::aruco::DetectorParameters> parameters = cv::aruco::DetectorParameters::create();
        cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_APRILTAG_36h11);

        // Detect tag and estimate pose
        cv::aruco::detectMarkers(image_ptr->image, dictionary, markerCorners, markerIds, parameters);
        cv::aruco::estimatePoseSingleMarkers(markerCorners, tagSize, cameraMatrix, distortionCoefficients, rvecs, tvecs);

        for (std::vector<int>::size_type i = 0; i < markerIds.size(); i++) {

            cv::aruco::drawDetectedMarkers(image_ptr->image, markerCorners, markerIds);
            cv::aruco::drawAxis(image_ptr->image, cameraMatrix, distortionCoefficients, rvecs[i], tvecs[i], 0.1);

            if (markerIds[i] == tagId) {

                setDetected(true);
                z_coord = calculateDistance(rvecs[i], tvecs[i]);

                /*
                cv::Mat r_mat;
                cv::Mat t_mat = cv::Mat(tvecs[i]).reshape(1);
                cv::Rodrigues(rvecs[i], r_mat);
                cv::Mat inversePose = r_mat.inv() * -t_mat;

                // y and z are flipped, since y is up and down movement of camera and z represents depth from tag
                x_coord = inversePose.at<double>(0, 0);
                y_coord = inversePose.at<double>(1, 0);
                z_coord = inversePose.at<double>(2, 0);
                 */
            }
        }

        /* For debugging
        cv::imshow("Detected Markers", image_ptr->image);
        cv::waitKey(0); // Wait for a key event to close the window
         */

    }

    double calculateDistance(cv::Vec3d& rvec, cv::Vec3d& tvec) {

        double distance;

        cv::Mat r_mat;

        // Convert vector to matrix format (for multiplication)
        cv::Mat t_mat = cv::Mat(tvec).reshape(1);

        // Convert 3D rotation vector to matrix using rodrigues function
        cv::Rodrigues(rvec, r_mat);

        // Inverse both rotation matrix and translation matrix
        cv::Mat inversePose = r_mat.inv() * -t_mat;

        // Get z value from matrix, (x, y, z) format
        distance = inversePose.at<double>(2,0);

        return distance;
    }

    AprilTag(const sensor_msgs::msg::Image& image, const double size, const int id) : tagId(id), tagSize(size), inputImage(image) {
    }

    void setDetected(bool value) {
        isDetected = value;
    }
    bool getDetected() {
        return isDetected;
    }

private:
    bool isDetected;
};