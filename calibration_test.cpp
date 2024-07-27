#include <opencv2/opencv.hpp>
#include <vector>
#include <iostream>

int main() {
    // Load calibration parameters
    cv::FileStorage fs("calibration.yml", cv::FileStorage::READ);
    cv::Mat cameraMatrix, distCoeffs;
    fs["cameraMatrix"] >> cameraMatrix;
    fs["distCoeffs"] >> distCoeffs;
    fs.release();

    // Load an example image
    std::string filename = "thermal_image_1.png"; // Change to any of your images
    cv::Mat image = cv::imread(filename, cv::IMREAD_COLOR);
    if (image.empty()) {
        std::cerr << "Could not open or find the image " << filename << std::endl;
        return -1;
    }

    // Undistort the image
    cv::Mat undistortedImage;
    cv::undistort(image, undistortedImage, cameraMatrix, distCoeffs);

    // Display the original and undistorted images
    cv::imshow("Original Image", image);
    cv::imshow("Undistorted Image", undistortedImage);
    cv::waitKey(0);

    // Save the undistorted image for comparison
    cv::imwrite("undistorted_image.png", undistortedImage);

    return 0;
}
