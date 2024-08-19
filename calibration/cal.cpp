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
    // Define the size of the chessboard pattern (number of inner corners per a chessboard row and column)
    Size patternSize(4, 5); // This needs to be adjusted to your pattern
    vector<vector<Point2f>> imagePoints;
    vector<vector<Point3f>> objectPoints;

    vector<Point3f> obj;
    for (int i = 0; i < patternSize.height; i++) {
        for (int j = 0; j < patternSize.width; j++) {
            obj.push_back(Point3f((float)j, (float)i, 0));
        }
    }

    // Load images
    vector<String> images;
    glob("image_set_5/gray/*.png", images); // Add the path to your thermal images

    for (size_t i = 0; i < images.size(); i++) {
        Mat image = imread(images[i], IMREAD_GRAYSCALE);
        if (image.empty()) {
            cout << "Cannot open image file: " << images[i] << endl;
            continue;
        }

        vector<Point2f> corners;
        bool patternFound = findChessboardCorners(image, patternSize, corners, CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_NORMALIZE_IMAGE);

        if (patternFound) {
            cornerSubPix(image, corners, Size(11, 11), Size(-1, -1), TermCriteria(TermCriteria::EPS + TermCriteria::COUNT, 30, 0.1));
            drawChessboardCorners(image, patternSize, Mat(corners), patternFound);
            imshow("Chessboard Pattern", image);

            int key = waitKey(0);
            if (key != 'n') {
                imagePoints.push_back(corners);
                objectPoints.push_back(obj);
            }
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
    FileStorage fs("calibration.xml", FileStorage::WRITE);
    fs << "cameraMatrix" << cameraMatrix;
    fs << "distCoeffs" << distCoeffs;
    fs.release();

    return 0;
}
