#include <opencv2/opencv.hpp>
#include <vector>
#include <iostream>

std::vector<cv::Point2f> detectSquareMesh(cv::Mat& image);
std::vector<cv::Point3f> createWorldCoordinates(cv::Size patternSize);

int main() {
    std::vector<cv::Mat> calibrationImages;
    for (int i = 1; i <= 19; ++i) {
        std::string filename = "thermal_image_" + std::to_string(i) + ".png";
        cv::Mat image = cv::imread(filename, cv::IMREAD_COLOR);
        if (image.empty()) {
            std::cerr << "Could not open or find the image " << filename << std::endl;
            return -1;
        }
        calibrationImages.push_back(image);
    }

    cv::Size patternSize(4, 4);
    std::vector<std::vector<cv::Point2f>> imagePoints;
    for (auto& img : calibrationImages) {
        cv::Mat gray;
        cv::imshow("Image", img);
        cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);
        cv::imshow("Gray Image", gray);
        std::vector<cv::Point2f> corners = detectSquareMesh(gray);

        if (corners.size() == patternSize.area()) {
            imagePoints.push_back(corners);
            // Draw detected corners on the image
            for (const auto& corner : corners) {
                cv::circle(img, corner, 5, cv::Scalar(0, 255, 0), -1);
            }
        } else {
            std::cerr << "Failed to detect the correct number of corners in one of the images." << std::endl;
            std::cerr << "Detected " << corners.size() << " corners, but expected " << patternSize.area() << " corners." << std::endl;
        }

        // Display the processed image
        cv::imshow("Processed Image", img);
        cv::waitKey(0);
    }

    if (imagePoints.size() < calibrationImages.size()) {
        std::cerr << "Not all images provided the required number of points. Calibration may not be accurate." << std::endl;
    }

    if (imagePoints.empty()) {
        std::cerr << "No valid images were detected. Exiting..." << std::endl;
        return -1;
    }

    std::vector<cv::Point3f> objectPoints = createWorldCoordinates(patternSize);
    std::vector<std::vector<cv::Point3f>> objectPointsVec(imagePoints.size(), objectPoints);

    cv::Mat cameraMatrix, distCoeffs;
    std::vector<cv::Mat> rvecs, tvecs;
    double rms = cv::calibrateCamera(objectPointsVec, imagePoints, calibrationImages[0].size(), cameraMatrix, distCoeffs, rvecs, tvecs);

    std::cout << "RMS error: " << rms << std::endl;
    std::cout << "Camera Matrix: " << cameraMatrix << std::endl;
    std::cout << "Distortion Coefficients: " << distCoeffs << std::endl;

    cv::FileStorage fs("calibration.yml", cv::FileStorage::WRITE);
    fs << "cameraMatrix" << cameraMatrix;
    fs << "distCoeffs" << distCoeffs;
    fs.release();

    return 0;
}

std::vector<cv::Point2f> detectSquareMesh(cv::Mat& image) {
    // Step 1: Adaptive thresholding
    cv::Mat thresh;
    cv::threshold(image,thresh,70,255,cv::THRESH_BINARY_INV);
    // cv::adaptiveThreshold(image, thresh, 255, cv::ADAPTIVE_THRESH_GAUSSIAN_C, cv::THRESH_BINARY, 11, 2);
    cv::imshow("Thresholded Image", thresh);
    cv::waitKey(0);

    // Step 2: Morphological operations (dilation followed by erosion)
    cv::Mat morph;
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(1, 1));
    cv::morphologyEx(thresh, morph, cv::MORPH_CLOSE, kernel);
    cv::morphologyEx(thresh, morph, cv::MORPH_CLOSE, kernel);
    kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
    cv::morphologyEx(thresh, morph, cv::MORPH_CLOSE, kernel);
    cv::imshow("Morphological Operations", morph);
    cv::waitKey(0);

    // Step 3: Contour detection
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(morph, contours, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
    cv::Mat contourImage = cv::Mat::zeros(image.size(), CV_8UC3);
    cv::drawContours(contourImage, contours, -1, cv::Scalar(255, 255, 255), 1);
    cv::imshow("Contours", contourImage);
    cv::waitKey(0);

    // Step 4: Detect intersections of contours
    std::vector<cv::Point2f> intersections;
    for (const auto& contour : contours) {
        if (contour.size() < 4) continue;
        for (size_t i = 0; i < contour.size(); i++) {
            for (size_t j = i + 1; j < contour.size(); j++) {
                if (cv::norm(contour[i] - contour[j]) < 2) {
                    intersections.push_back((contour[i] + contour[j]) * 0.5);
                }
            }
        }
    }

    std::vector<cv::Point2f> corners;
    cv::goodFeaturesToTrack(morph, corners, 16, 0.01, 10);
    cv::cornerSubPix(morph, corners, cv::Size(11, 11), cv::Size(-1, -1), 
                     cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.01));
    
    // Step 5: Draw detected intersections
    // cv::Mat intersectionsImage = cv::Mat::zeros(image.size(), CV_8UC3);
    // for (const auto& corner : corners) {
    //     cv::circle(intersectionsImage, corner, 5, cv::Scalar(0, 0, 255), -1);
    // }

    // Step 5: Draw detected intersections
    cv::Mat intersectionsImage = cv::Mat::zeros(image.size(), CV_8UC3);
    for (const auto& intersection : intersections) {
        cv::circle(intersectionsImage, intersection, 5, cv::Scalar(0, 0, 255), -1);
    }
    cv::imshow("Intersections", intersectionsImage);
    cv::waitKey(0);

    // Sort the detected points to ensure they are ordered consistently
    std::sort(intersections.begin(), intersections.end(), [](const cv::Point2f& a, const cv::Point2f& b) {
        return a.y < b.y || (a.y == b.y && a.x < b.x);
    });

    return intersections;
}

std::vector<cv::Point3f> createWorldCoordinates(cv::Size patternSize) {
    std::vector<cv::Point3f> objectPoints;
    for (int i = 0; i < patternSize.height; ++i) {
        for (int j = 0; j < patternSize.width; ++j) {
            objectPoints.emplace_back(j, i, 0.0f);
        }
    }
    return objectPoints;
}


#include <opencv2/opencv.hpp>
#include <vector>
#include <iostream>

std::vector<cv::Point2f> detectSquareMesh(cv::Mat& image);
std::vector<cv::Point3f> createWorldCoordinates(cv::Size patternSize);

int main() {
    std::vector<cv::Mat> calibrationImages;
    for (int i = 1; i <= 19; ++i) {
        std::string filename = "thermal_image_" + std::to_string(i) + ".png";
        cv::Mat image = cv::imread(filename, cv::IMREAD_COLOR);
        if (image.empty()) {
            std::cerr << "Could not open or find the image " << filename << std::endl;
            return -1;
        }
        calibrationImages.push_back(image);
    }

    cv::Size patternSize(4, 4); // Adjust based on your mesh pattern size
    std::vector<std::vector<cv::Point2f>> imagePoints;
    for (auto& img : calibrationImages) {
        cv::Mat gray;
        cv::imshow("Image", img);
        cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);
        cv::imshow("Gray Image", gray);
        std::vector<cv::Point2f> corners = detectSquareMesh(gray);

        if (corners.size() == patternSize.area()) {
            imagePoints.push_back(corners);
            // Draw detected corners on the image
            for (const auto& corner : corners) {
                cv::circle(img, corner, 5, cv::Scalar(0, 255, 0), -1);
            }
        } else {
            std::cerr << "Failed to detect the correct number of corners in one of the images." << std::endl;
            std::cerr << "Detected " << corners.size() << " corners, but expected " << patternSize.area() << " corners." << std::endl;
        }

        // Display the processed image
        // cv::imshow("Processed Image", img);
        // cv::waitKey(0);
    }

    if (imagePoints.size() < calibrationImages.size()) {
        std::cerr << "Not all images provided the required number of points. Calibration may not be accurate." << std::endl;
    }

    if (imagePoints.empty()) {
        std::cerr << "No valid images were detected. Exiting..." << std::endl;
        return -1;
    }

    std::vector<cv::Point3f> objectPoints = createWorldCoordinates(patternSize);
    std::vector<std::vector<cv::Point3f>> objectPointsVec(imagePoints.size(), objectPoints);

    cv::Mat cameraMatrix, distCoeffs;
    std::vector<cv::Mat> rvecs, tvecs;
    double rms = cv::calibrateCamera(objectPointsVec, imagePoints, calibrationImages[0].size(), cameraMatrix, distCoeffs, rvecs, tvecs);

    std::cout << "RMS error: " << rms << std::endl;
    std::cout << "Camera Matrix: " << cameraMatrix << std::endl;
    std::cout << "Distortion Coefficients: " << distCoeffs << std::endl;

    cv::FileStorage fs("calibration.yml", cv::FileStorage::WRITE);
    fs << "cameraMatrix" << cameraMatrix;
    fs << "distCoeffs" << distCoeffs;
    fs.release();

    return 0;
}

std::vector<cv::Point2f> detectSquareMesh(cv::Mat& image) {
    // Step 1: Preprocessing
    cv::Mat blurred, equalized;
    cv::GaussianBlur(image, blurred, cv::Size(5, 5), 0);
    cv::equalizeHist(blurred, equalized);
    cv::imshow("Equalized Image", equalized);
    cv::waitKey(0);

    // Step 2: Adaptive thresholding
    cv::Mat thresh;
    cv::threshold(equalized,thresh,70,255,cv::THRESH_BINARY_INV);
    // cv::adaptiveThreshold(image, thresh, 255, cv::ADAPTIVE_THRESH_GAUSSIAN_C, cv::THRESH_BINARY, 11, 2);
    cv::imshow("Thresholded Image", thresh);
    cv::waitKey(0);
    // cv::Mat thresh;
    // cv::adaptiveThreshold(equalized, thresh, 255, cv::ADAPTIVE_THRESH_GAUSSIAN_C, cv::THRESH_BINARY_INV, 11, 2);
    // cv::imshow("Thresholded Image", thresh);
    // cv::waitKey(0);

    // Step 3: Morphological operations (dilation followed by erosion)
    // cv::Mat morph;
    // cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
    // cv::morphologyEx(thresh, morph, cv::MORPH_CLOSE, kernel);
    // cv::imshow("Morphological Operations", morph);
    // cv::waitKey(0);
    cv::Mat morph;
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(1, 1));
    cv::morphologyEx(thresh, morph, cv::MORPH_CLOSE, kernel);
    cv::morphologyEx(thresh, morph, cv::MORPH_CLOSE, kernel);
    kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
    cv::morphologyEx(thresh, morph, cv::MORPH_CLOSE, kernel);
    cv::imshow("Morphological Operations", morph);
    cv::waitKey(0);

    // Step 4: Corner detection using Shi-Tomasi
    std::vector<cv::Point2f> corners;
    cv::goodFeaturesToTrack(morph, corners, 16, 0.01, 10);
    cv::cornerSubPix(morph, corners, cv::Size(11, 11), cv::Size(-1, -1), 
                     cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.01));
    
    // Step 5: Draw detected intersections
    cv::Mat intersectionsImage = cv::Mat::zeros(image.size(), CV_8UC3);
    for (const auto& corner : corners) {
        cv::circle(intersectionsImage, corner, 5, cv::Scalar(0, 0, 255), -1);
    }
    cv::imshow("Intersections", intersectionsImage);
    cv::waitKey(0);

    // Print out intersection points for debugging
    std::cout << "Intersections detected: " << corners.size() << std::endl;
    for (const auto& corner : corners) {
        std::cout << "(" << corner.x << ", " << corner.y << ")" << std::endl;
    }

    // Sort the detected points to ensure they are ordered consistently
    std::sort(corners.begin(), corners.end(), [](const cv::Point2f& a, const cv::Point2f& b) {
        return a.y < b.y || (a.y == b.y && a.x < b.x);
    });

    return corners;
}

std::vector<cv::Point3f> createWorldCoordinates(cv::Size patternSize) {
    std::vector<cv::Point3f> objectPoints;
    for (int i = 0; i < patternSize.height; ++i) {
        for (int j = 0; j < patternSize.width; ++j) {
            objectPoints.emplace_back(j, i, 0.0f);
        }
    }
    return objectPoints;
}
