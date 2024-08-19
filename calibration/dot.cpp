#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>
#include <vector>

using namespace cv;
using namespace std;

// Function to detect the dot pattern
bool findDotPattern(const Mat& image, Size patternSize, vector<Point2f>& centers) {
    SimpleBlobDetector::Params params;
    // params.minThreshold = 127;
    // params.maxThreshold = 255;
    params.filterByArea = true;
    params.minArea = 2;
    params.maxArea = 10000;
    params.filterByCircularity = true;
    params.minCircularity = 0.5;
    // params.filterByConvexity = true;
    // params.minConvexity = 0.7;
    // params.filterByInertia = true;
    // params.minInertiaRatio = 0.1;

    Ptr<SimpleBlobDetector> detector = SimpleBlobDetector::create(params);

    Mat thresholded;
    adaptiveThreshold(image, thresholded, 255, ADAPTIVE_THRESH_GAUSSIAN_C, THRESH_BINARY, 13, 2);

    // Debug: show the thresholded image
    imshow("Thresholded Image", thresholded);
    waitKey(0);

    return findCirclesGrid(thresholded, patternSize, centers, CALIB_CB_ASYMMETRIC_GRID, detector);
}

int main() {
    // Define the size of the dot pattern (number of inner corners per a row and column)
    Size patternSize(2, 7); // Adjust to your pattern
    vector<vector<Point2f>> imagePoints;
    vector<vector<Point3f>> objectPoints;

    vector<Point3f> obj;
    for (int i = 0; i < patternSize.height; i++) {
        for (int j = 0; j < patternSize.width; j++) {
            obj.push_back(Point3f((2 * j + i % 2), i, 0));
        }
    }

    // Load images
    vector<String> images;
    glob("image_set_6/gray/*.png", images); // Add the path to your thermal images

    for (size_t i = 0; i < images.size(); i++) {
        Mat image = imread(images[i], IMREAD_COLOR);
        if (image.empty()) {
            cout << "Cannot open image file: " << images[i] << endl;
            continue;
        }
        Mat gray;
        cvtColor(image, gray, COLOR_BGR2GRAY);
        // Enhance image contrast
        Mat enhancedImage;
        equalizeHist(gray, enhancedImage);
        imshow("Enhanced", enhancedImage);
        waitKey(0);

        vector<Point2f> centers;
        bool patternFound = findDotPattern(enhancedImage, patternSize, centers);

        if (patternFound) {
            imagePoints.push_back(centers);
            objectPoints.push_back(obj);

            drawChessboardCorners(image, patternSize, Mat(centers), patternFound);
            imshow("Dot Pattern", image);
            waitKey(0);
        } else {
            cout << "Pattern not found in image: " << images[i] << endl;
        }
    }

    destroyAllWindows();

    // Camera calibration
    Mat cameraMatrix, distCoeffs;
    vector<Mat> rvecs, tvecs;
    calibrateCamera(objectPoints, imagePoints, patternSize, cameraMatrix, distCoeffs, rvecs, tvecs);

    cout << "Camera matrix: \n" << cameraMatrix << endl;
    cout << "Distortion coefficients: \n" << distCoeffs << endl;

    // Save calibration parameters
    FileStorage fs("calibration_dot.xml", FileStorage::WRITE);
    fs << "cameraMatrix" << cameraMatrix;
    fs << "distCoeffs" << distCoeffs;
    fs.release();

    return 0;
}
