#include <opencv2/opencv.hpp>
#include <vector>

/**
 * @brief Sample code for camera calibration using OpenCV.
 * @details This program demonstrates camera calibration using a checkerboard pattern.
 * It reads images, finds checkerboard corners, and performs camera calibration
 * using the cv::calibrateCamera function. The cameraMatrix and distortionCoeffs
 * obtained from this calibration can be used for pose estimation.
 *
 * @author Grayson Arendt
 */

/**
 * @brief Main function for camera calibration.
 */
int main()
{
    // Define the size of the checkerboard (inner corners).
    cv::Size boardSize(10, 7); // 11x8 checkerboard

    // Define the size of each square in millimeters.
    float squareSize = 25.0; // 25mm squares

    int numImages = 10;

    cv::Size imageSize(640, 480);

    // Create vectors to store object points and image points from all images.
    std::vector<std::vector<cv::Point3f>> objectPoints;
    std::vector<std::vector<cv::Point2f>> imagePoints;

    // Read images and find checkerboard corners.
    for (int i = 1; i <= numImages; ++i)
    {
        // Load the image.
        std::string imagePath = "/home/tablet/lunabot_ws/src/Lunabotics-2024/autonomous/localization_pkg/images/image" + std::to_string(i) + ".png";
        cv::Mat image = cv::imread(imagePath, cv::IMREAD_GRAYSCALE);

        // Find chessboard corners.
        std::vector<cv::Point2f> corners;
        bool patternFound = cv::findChessboardCorners(image, boardSize, corners);

        if (patternFound)
        {
            // If the corners are found, refine them.
            cv::cornerSubPix(image, corners, cv::Size(11, 11), cv::Size(-1, -1),
                             cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.1));
            // Add object points (world coordinates) and image points (pixel coordinates).
            std::vector<cv::Point3f> obj;
            for (int y = 0; y < boardSize.height; ++y)
            {
                for (int x = 0; x < boardSize.width; ++x)
                {
                    obj.push_back(cv::Point3f(x * squareSize, y * squareSize, 0));
                }
            }
            objectPoints.push_back(obj);
            imagePoints.push_back(corners);

            // Draw and display the corners (optional).
            // cv::drawChessboardCorners(image, boardSize, corners, patternFound);
            // cv::imshow("Chessboard Corners", image);
            // cv::waitKey(500); // Adjust the wait time as needed.
        }
    }

    // Calibrate the camera using cv::calibrateCamera function.
    cv::Mat cameraMatrix, distCoeffs;
    std::vector<cv::Mat> rvecs, tvecs;
    cv::calibrateCamera(objectPoints, imagePoints, imageSize, cameraMatrix, distCoeffs, rvecs, tvecs);

    // Print calibration results
    std::cout << "Camera Matrix:" << std::endl
              << cameraMatrix << std::endl;
    std::cout << "Distortion Coefficients:" << std::endl
              << distCoeffs << std::endl;

    return 0;
}
