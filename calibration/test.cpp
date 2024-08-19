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
    cv::Mat BGRImage = cv::imread("extrinsic_images/thermal_image_3.png", cv::IMREAD_COLOR);
    cv::Mat GrayImage = cv::imread("extrinsic_images/thermal_grayimage_3.png", cv::IMREAD_GRAYSCALE);
    cv::Mat gray;

    cv::cvtColor(BGRImage, gray, COLOR_BGR2GRAY);

    cv::imshow("Thermal Raw", GrayImage);
    cv::imshow("Thermal RGB", BGRImage);
    cv::imshow("Thermal Gray", gray);
    cv::waitKey(0);

    Size patternSize(4, 5); // This needs to be adjusted to your pattern
    vector<vector<Point2f>> imagePoints;
    vector<vector<Point3f>> objectPoints;

    vector<Point3f> obj;
    for (int i = 0; i < patternSize.height; i++) {
        for (int j = 0; j < patternSize.width; j++) {
            obj.push_back(Point3f((float)j, (float)i, 0));
        }
    }

    vector<Point2f> corners;
    bool patternFound = findChessboardCorners(GrayImage, patternSize, corners, CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_NORMALIZE_IMAGE);
    int key;

    if (patternFound) {
        cornerSubPix(GrayImage, corners, Size(11, 11), Size(-1, -1), TermCriteria(TermCriteria::EPS + TermCriteria::COUNT, 30, 0.1));
        drawChessboardCorners(GrayImage, patternSize, Mat(corners), patternFound);
        imshow("Chessboard Pattern", GrayImage);

        key = waitKey(0);
        if (key != 'n') {
            imagePoints.push_back(corners);
            objectPoints.push_back(obj);
        }
    } else {
        cout << "Pattern not found in image:1 " << endl;
    }

    patternFound = findChessboardCorners(BGRImage, patternSize, corners, CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_NORMALIZE_IMAGE);

    if (patternFound) {
        cornerSubPix(BGRImage, corners, Size(11, 11), Size(-1, -1), TermCriteria(TermCriteria::EPS + TermCriteria::COUNT, 30, 0.1));
        drawChessboardCorners(BGRImage, patternSize, Mat(corners), patternFound);
        imshow("Chessboard Pattern", BGRImage);

        key = waitKey(0);
        if (key != 'n') {
            imagePoints.push_back(corners);
            objectPoints.push_back(obj);
        }
    } else {
        cout << "Pattern not found in image:2 " << endl;
    }

    patternFound = findChessboardCorners(gray, patternSize, corners, CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_NORMALIZE_IMAGE);

    if (patternFound) {
        cornerSubPix(gray, corners, Size(11, 11), Size(-1, -1), TermCriteria(TermCriteria::EPS + TermCriteria::COUNT, 30, 0.1));
        drawChessboardCorners(gray, patternSize, Mat(corners), patternFound);
        imshow("Chessboard Pattern", gray);

        key = waitKey(0);
        if (key != 'n') {
            imagePoints.push_back(corners);
            objectPoints.push_back(obj);
        }
    } else {
        cout << "Pattern not found in image:3 " << endl;
    }

    destroyAllWindows();
    return 0;
}
