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

This node detects AprilTags from the 36h11 family using OpenCV 4.2,
creates an image with marker, id name, and pose overlays,
then calculates how far the tag is from the camera. 
*/

class aprilTagDetection : public rclcpp::Node {
    public:
    aprilTagDetection() : Node("apriltagDetector") {

        // Create subscriber
        imageSub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "camera/color/image_raw", 10,
            [this](const sensor_msgs::msg::Image::SharedPtr image) {
            
            // Detect apriltags and distance from each image
            detectAprilTag(image);
            }
        );

        // Create publisher
        overlayPub_ = this->create_publisher<sensor_msgs::msg::Image>("overlay_image", 10);
    }

    private:

        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr imageSub_;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr overlayPub_;
                
        void detectAprilTag(const sensor_msgs::msg::Image::SharedPtr inputImage) {

            // Convert inputImage to an opencv matrix 
            cv_bridge::CvImagePtr currentImage_ptr = cv_bridge::toCvCopy(inputImage, inputImage->encoding);

            // Clone image for future overlay image
            cv::Mat outputImage = currentImage_ptr->image.clone();
            
            // Initialize vectors for markerIds and markerCorners
            std::vector<int> markerIds;
            std::vector<std::vector<cv::Point2f>> markerCorners;

            // Values from camera calibration
            cv::Mat cameraMatrix = (cv::Mat1d(3, 3) << 383.4185742519996, 0, 309.4326377845713, 0, 385.0909007102088, 240.749949733094, 0, 0, 1);
            cv::Mat distortionCoefficients = (cv::Mat1d(1, 5) << -0.06792929080519726, 0.08058277259698843, -0.001690544521662593, -0.0008235437909836152, -0.04417756393089296);

            // Rotation and translation vectors
            std::vector<cv::Vec3d> rvecs, tvecs;
           
            // Parameters and dictionary
            cv::Ptr<cv::aruco::DetectorParameters> parameters = cv::aruco::DetectorParameters::create();
            cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_APRILTAG_36h11);

            // Detect markers
            cv::aruco::detectMarkers(currentImage_ptr->image, dictionary, markerCorners, markerIds, parameters);

            // Estimate pose, need cameraMatrix and distCoeffs first from calibration
            cv::aruco::estimatePoseSingleMarkers(markerCorners, 0.06, cameraMatrix, distortionCoefficients, rvecs, tvecs);

             // Check to see if tags have been detected
            if (!markerIds.empty()) {

                for (std::vector<int>::size_type i = 0; i < markerIds.size(); i++) {

                        double currentDistance;
                        
                        // Draw pose axes on overlay image
                        cv::aruco::drawAxis(outputImage, cameraMatrix, distortionCoefficients, rvecs[i], tvecs[i], 0.1);

                        // Calculate distance in meters
                        currentDistance = calculateDistance(rvecs[i], tvecs[i]);

                        // Output distance and id
                        RCLCPP_INFO(this->get_logger(), "ID: %i Distance (m): %f", markerIds[i], currentDistance);
                }

                // Draw around detected markers
                cv::aruco::drawDetectedMarkers(outputImage, markerCorners, markerIds);
                
                // Convert back to ROS2 image
                auto msg_ = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", outputImage).toImageMsg();

                // Publish overlay image
                overlayPub_->publish(*msg_.get());
            } 

            else {
                // Output no tags
                RCLCPP_INFO(this->get_logger(), "No apriltags have been detected");
            }

        }

        /*
        We want to get the pure distance regardless of tag orientation (camera viewing at different angles),
        so we will now inverse the pose. Inversing the pose brings the object back to original orientation and position
        without including the camera's perspective/distortion, so the true distance would be 
        the z value of that original position.
        */

        double calculateDistance(cv::Vec3d& rvec, cv::Vec3d& tvec) {

            double distance;

            cv::Mat r_mat;
            
            // Convert vector to matrix format (for multiplication)
            cv::Mat t_mat = cv::Mat(tvec).reshape(1);
            
            // Convert 3D rotation vector to matrix using rodrigues function
            cv::Rodrigues(rvec, r_mat); 

            // Inverse both rotation matrix and translation matrix)
            cv::Mat inversePose = r_mat.inv() * -t_mat;

            // Get z value from matrix, (x, y, z) format
            distance = inversePose.at<double>(2,0);

            return distance;
        }

};

int main(int argc, char * argv[]) {

    // Start ROS2
    rclcpp::init(argc,argv);

    // Keep running node
    rclcpp::spin(std::make_shared<aprilTagDetection>());

    // Shutdown node
    rclcpp::shutdown();

    return 0;
}