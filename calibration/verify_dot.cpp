#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>

using namespace cv;
using namespace std;

int main() {
    // Load the calibration parameters
    Mat cameraMatrix, distCoeffs;
    FileStorage fs("../calibration.xml", FileStorage::READ); // Add the path to your calibration file
    if (!fs.isOpened()) {
        cout << "Failed to open calibration file" << endl;
        return -1;
    }
    fs["cameraMatrix"] >> cameraMatrix;
    fs["distCoeffs"] >> distCoeffs;
    fs.release();

    // Load an image to undistort
    Mat image = imread("image_set_6/thermal_image_7.png"); // Add the path to your thermal images
    if (image.empty()) {
        cout << "Cannot open image file" << endl;
        return -1;
    }

    // Undistort the image
    Mat undistortedImage;
    undistort(image, undistortedImage, cameraMatrix, distCoeffs);

    // Display the original and undistorted images
    namedWindow("Original Image", WINDOW_AUTOSIZE);
    imshow("Original Image", image);

    namedWindow("Undistorted Image", WINDOW_AUTOSIZE);
    imshow("Undistorted Image", undistortedImage);

    waitKey(0);
    destroyAllWindows();

    return 0;
}
