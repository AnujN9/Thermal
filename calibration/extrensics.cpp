#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>

using namespace cv;
using namespace std;

int main() {
    // Load intrinsic parameters from XML file
    FileStorage fs("calibration.xml", FileStorage::READ);
    Mat cameraMatrix, distCoeffs;
    fs["cameraMatrix"] >> cameraMatrix;
    fs["distCoeffs"] >> distCoeffs;
    fs.release();

    // Define the chessboard dimensions
    int boardWidth = 4;
    int boardHeight = 5;
    Size boardSize(boardWidth, boardHeight);

    // Prepare object points (chessboard corners in 3D space)
    vector<Point3f> objectPoints;
    for (int i = 0; i < boardHeight; i++) {
        for (int j = 0; j < boardWidth; j++) {
            objectPoints.push_back(Point3f(j, i, 0));
        }
    }

    // Capture or load calibration images
    vector<string> imageFiles = { "image_set_5/thermal_image_1.png", "image_set_5/thermal_image_2.png", "image_set_5/thermal_image_3.png" };
    vector<vector<Point2f>> imagePoints;
    Size imageSize;

    for (const string& file : imageFiles) {
        Mat image = imread(file);
        if (image.empty()) {
            cerr << "Error loading image: " << file << endl;
            return -1;
        }
        imageSize = image.size();

        vector<Point2f> corners;
        bool found = findChessboardCorners(image, boardSize, corners);
        if (found) {
            // Refine corner locations
            Mat gray;
            cvtColor(image, gray, COLOR_BGR2GRAY);
            cornerSubPix(gray, corners, Size(11, 11), Size(-1, -1),
                         TermCriteria(TermCriteria::EPS + TermCriteria::COUNT, 30, 0.1));
            imagePoints.push_back(corners);
        }
    }

    // Compute extrinsics for each image
    for (size_t i = 0; i < imagePoints.size(); i++) {
        Mat rvec, tvec;
        solvePnP(objectPoints, imagePoints[i], cameraMatrix, distCoeffs, rvec, tvec);

        // Convert rotation vector to rotation matrix
        Mat rotationMatrix;
        Rodrigues(rvec, rotationMatrix);

        // Print the extrinsics
        cout << "Image " << i + 1 << ":\n";
        cout << "Rotation Matrix:\n" << rotationMatrix << endl;
        cout << "Translation Vector:\n" << tvec << endl;
    }

    return 0;
}
