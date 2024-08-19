#include <opencv2/opencv.hpp>

// Function to load the thermal camera intrinsics
bool loadThermalCameraIntrinsics(const std::string &filename, cv::Mat &cameraMatrix, cv::Mat &distCoeffs) {
    cv::FileStorage fs(filename, cv::FileStorage::READ);
    if (!fs.isOpened()) {
        std::cerr << "Failed to open file: " << filename << std::endl;
        return false;
    }
    fs["cameraMatrix"] >> cameraMatrix;
    fs["distCoeffs"] >> distCoeffs;
    fs.release();
    return true;
}

int main() {
    // Load thermal camera intrinsics
    cv::Mat cameraMatrixThermal, distCoeffsThermal;
    if (!loadThermalCameraIntrinsics("../calibrate.xml", cameraMatrixThermal, distCoeffsThermal)) {
        return -1;
    }

    // Define the checkerboard size and square size (in meters)
    cv::Size boardSize(4, 5);
    float squareSize = 0.02f; // 20mm per square

    // Prepare 3D points for the checkerboard (object points)
    std::vector<cv::Point3f> objectPoints;
    for (int i = 0; i < boardSize.height; ++i) {
        for (int j = 0; j < boardSize.width; ++j) {
            objectPoints.push_back(cv::Point3f(j * squareSize, i * squareSize, 0));
        }
    }

    std::vector<std::vector<cv::Point2f>> imagePointsDepth, imagePointsThermal;
    std::vector<std::vector<cv::Point3f>> objectPointsAll;

    // Load the captured images
    std::vector<std::string> depthImages = {"../color_image_1.png", "../color_image_2.png", "../color_image_3.png"};  // Paths to depth camera images
    std::vector<std::string> thermalImages = {"../thermal_image_1.png", "../thermal_image_2.png", "../thermal_image_3.png"};  // Paths to thermal camera images

    for (size_t i = 0; i < depthImages.size(); ++i) {
        // Load the depth image
        cv::Mat depthImage = cv::imread(depthImages[i], cv::IMREAD_COLOR);
        if (depthImage.empty()) {
            std::cerr << "Failed to load depth image: " << depthImages[i] << std::endl;
            continue;
        }

        // Load the corresponding thermal image
        cv::Mat thermalImage = cv::imread(thermalImages[i], cv::IMREAD_GRAYSCALE); // Assuming thermal images are grayscale
        if (thermalImage.empty()) {
            std::cerr << "Failed to load thermal image: " << thermalImages[i] << std::endl;
            continue;
        }

        // Detect checkerboard corners in both images
        std::vector<cv::Point2f> cornersDepth, cornersThermal;
        bool foundDepth = cv::findChessboardCorners(depthImage, boardSize, cornersDepth);
        bool foundThermal = cv::findChessboardCorners(thermalImage, boardSize, cornersThermal);

        if (foundDepth && foundThermal) {
            // Refine corner positions
            cv::cornerSubPix(depthImage, cornersDepth, cv::Size(11, 11), cv::Size(-1, -1),
                             cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.01));
            cv::cornerSubPix(thermalImage, cornersThermal, cv::Size(11, 11), cv::Size(-1, -1),
                             cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.01));

            // Store points
            imagePointsDepth.push_back(cornersDepth);
            imagePointsThermal.push_back(cornersThermal);
            objectPointsAll.push_back(objectPoints);

            // Show detected corners
            cv::drawChessboardCorners(depthImage, boardSize, cornersDepth, foundDepth);
            cv::drawChessboardCorners(thermalImage, boardSize, cornersThermal, foundThermal);

            cv::imshow("Depth Camera Image", depthImage);
            cv::imshow("Thermal Camera Image", thermalImage);
            cv::waitKey(500); // Display for half a second
        }
    }

    // Perform stereo calibration
    cv::Mat cameraMatrixDepth, distCoeffsDepth, R, T, E, F;
    double rms = cv::stereoCalibrate(objectPointsAll, imagePointsDepth, imagePointsThermal,
                                     cameraMatrixDepth, distCoeffsDepth,
                                     cameraMatrixThermal, distCoeffsThermal,
                                     cv::Size(depthImages[0].size().width, depthImages[0].size().height),
                                     R, T, E, F,
                                     cv::CALIB_FIX_INTRINSIC,
                                     cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 100, 1e-5));

    std::cout << "Stereo calibration RMS error: " << rms << std::endl;
    std::cout << "Rotation matrix: " << R << std::endl;
    std::cout << "Translation vector: " << T << std::endl;

    // Save the extrinsic parameters
    cv::FileStorage fs("extrinsics.xml", cv::FileStorage::WRITE);
    fs << "R" << R;
    fs << "T" << T;
    fs.release();

    return 0;
}
