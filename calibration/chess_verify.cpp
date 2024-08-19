#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>
#include <vector>

using namespace cv;
using namespace std;

int main() {
    // Load the calibration parameters
    Mat cameraMatrix, distCoeffs;
    FileStorage fs("calibration.xml", FileStorage::READ); // Add the path to your calibration file
    if (!fs.isOpened()) {
        cout << "Failed to open calibration file" << endl;
        return -1;
    }
    fs["cameraMatrix"] >> cameraMatrix;
    fs["distCoeffs"] >> distCoeffs;
    fs.release();

    // Load all images used in the calibration
    vector<String> images;
    glob("image_set_5/*.png", images); // Add the path to your thermal images

    Size patternSize(4, 5); // Adjust this to your actual chessboard pattern size
    vector<vector<Point2f>> imagePoints;
    vector<vector<Point3f>> objectPoints;

    vector<Point3f> obj;
    for (int i = 0; i < patternSize.height; i++) {
        for (int j = 0; j < patternSize.width; j++) {
            obj.push_back(Point3f((float)j, (float)i, 0));
        }
    }

    // Initialize the error sum
    double totalError = 0;
    int totalPoints = 0;

    for (size_t i = 0; i < images.size(); i++) {
        Mat image = imread(images[i]);
        if (image.empty()) {
            cout << "Cannot open image file: " << images[i] << endl;
            continue;
        }

        Mat gray;
        cvtColor(image, gray, COLOR_BGR2GRAY);

        // Detect the corners
        vector<Point2f> corners;
        bool patternFound = findChessboardCorners(gray, patternSize, corners, CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_NORMALIZE_IMAGE);

        if (patternFound) {
            // Refine corner locations
            cornerSubPix(gray, corners, Size(11, 11), Size(-1, -1), TermCriteria(TermCriteria::EPS + TermCriteria::COUNT, 30, 0.1));

            // Compute the re-projection error
            vector<Point2f> reprojectedPoints;
            Mat rvec, tvec;
            solvePnP(obj, corners, cameraMatrix, distCoeffs, rvec, tvec);
            projectPoints(obj, rvec, tvec, cameraMatrix, distCoeffs, reprojectedPoints);

            double error = norm(Mat(corners), Mat(reprojectedPoints), NORM_L2);
            totalError += error * error;
            totalPoints += patternSize.area();

            // Display the original and undistorted images
            Mat undistortedImage;
            undistort(image, undistortedImage, cameraMatrix, distCoeffs);

            String windowOriginal = "Original Image " + to_string(i + 1);
            namedWindow(windowOriginal, WINDOW_AUTOSIZE);
            imshow(windowOriginal, image);

            String windowUndistorted = "Undistorted Image " + to_string(i + 1);
            namedWindow(windowUndistorted, WINDOW_AUTOSIZE);
            imshow(windowUndistorted, undistortedImage);

            waitKey(0); // Wait for a key press before moving to the next image
            destroyWindow(windowOriginal);
            destroyWindow(windowUndistorted);
        } else {
            cout << "Pattern not found in image: " << images[i] << endl;
        }
    }

    // Calculate RMS error
    double rmsError = sqrt(totalError / totalPoints);
    cout << "RMS Re-projection Error: " << rmsError << endl;

    return 0;
}
