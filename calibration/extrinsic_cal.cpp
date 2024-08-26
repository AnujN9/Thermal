#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>
#include <iostream>
#include <vector>
#include <string>

/// \file depthimage.cpp
/// \brief Program that streams thermal images and saves it to thermal_images directory.

/// \brief Function to describe how to use the command line arguments
/// \param cmd Argument of the command line, here it is the program
void printUsage(char *cmd)
{
	char *cmdname = basename(cmd);
	printf(" Usage: %s [OPTION]...\n"
		   " -h             display this help and exit.\n"
		   " -r x		number of rows in the patterns (default 4).\n"
		   " -c x		number of columns in the patterns (default 5).\n"
           " -n x		number of image pairs (NEEDED).\n"
		   " Output:	Camera extrinsics between the thermal and rgb\n"
		   "                camera saved to an extrinsic.xml file.\n"
		   "", cmdname);
	return;
}

/// \brief Finds the pattern between thermal and color images to determine the extrinsics.
/// \param argc Number of command-line arguments.
/// \param argv Array of command-line arguments.
/// \param r Number of rows in the patterns.
/// \param c Number of columns in the patterns.
/// \param n Number of images pairs.
/// \return 0 if successful, -1 if failure.
int main(int argc, char **argv)
{
    int row = 4;
    int column = 5;
    int n = -1;
    for(int i=1; i < argc; i++)
	{
		if (std::strcmp(argv[i], "-h") == 0)
		{
			printUsage(argv[0]);
			exit(0);
		}
		else if (std::strcmp(argv[i], "-r") == 0)
		{
			if (i + 1 != argc)
			{
				int temp = std::stoi(argv[++i]);
				if (temp < 0 || temp > 255){
					std::cerr << "Error: Enter a valid number of rows." << std::endl;
					exit(1);
				}
				row = temp;
			}
            else
            {
				std::cerr << "Error: Enter the number of rows." << std::endl;
				exit(1);
			}
		}
		else if (std::strcmp(argv[i], "-c") == 0)
		{
			if (i + 1 != argc)
			{
				int temp = std::stoi(argv[++i]);
				if (temp < 0 || temp > 255){
					std::cerr << "Error: Enter a valid number of columns." << std::endl;
					exit(1);
				}
				column = temp;
			}
            else
            {
				std::cerr << "Error: Enter the number of columns." << std::endl;
				exit(1);
			}
		}
        else if (std::strcmp(argv[i], "-n") == 0)
		{
			if (i + 1 != argc)
			{
				int temp = std::stoi(argv[++i]);
				if (temp < 0 || temp > 255){
					std::cerr << "Error: Enter a valid number of images." << std::endl;
					exit(1);
				}
				n = temp;
			}
            else
            {
				std::cerr << "Error: Enter the number of columns." << std::endl;
				exit(1);
			}
		}
	}

    if (n == -1)
    {
        std::cerr << "Error: Enter the number of images. Check options with -h" << std::endl;
        exit(1);
    }
    std::string imageDirectory = IMAGE_DIRECTORY;
    std::string thermalimageDirectory = THERMAL_IMAGE_DIRECTORY;
    std::string calibrationfile = imageDirectory + "/../calibration.xml";
    cv::FileStorage fs(calibrationfile, cv::FileStorage::READ);    
    cv::Mat cameraMatrixThermal, distCoeffsThermal;
    fs["cameraMatrix"] >> cameraMatrixThermal;
    fs["distCoeffs"] >> distCoeffsThermal;
    fs.release();

    rs2::pipeline pipe;
    pipe.start();
    auto color_stream = pipe.get_active_profile().get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>();
    rs2_intrinsics intrinsicsColor = color_stream.get_intrinsics();
    cv::Mat cameraMatrixRealSense = (cv::Mat_<double>(3, 3) <<
        intrinsicsColor.fx, 0, intrinsicsColor.ppx,
        0, intrinsicsColor.fy, intrinsicsColor.ppy,
        0, 0, 1);

    cv::Mat distCoeffsRealSense = cv::Mat::zeros(5, 1, CV_64F);
    for (int i = 0; i < 5; ++i)
    {
        distCoeffsRealSense.at<double>(i, 0) = intrinsicsColor.coeffs[i];
    }

    // Set up chessboard parameters
    double squareSize = 0.03;  // Square size in meters (30mm)
    std::vector<cv::Point3f> objectPoints;
    cv::Size patternSize(row, column); // Chessboard size
    for (int i = 0; i < patternSize.height; ++i)
    {
        for (int j = 0; j < patternSize.width; ++j)
        {
            objectPoints.emplace_back(j * squareSize, i * squareSize, 0);
        }
    }
    std::vector<std::vector<cv::Point2f>> cornersColorVec, cornersThermalVec;
    std::vector<std::vector<cv::Point3f>> objectPointsVec;

    for (int imgIndex = 1; imgIndex <= n; ++imgIndex)
    {
        std::string colorImagePath = imageDirectory + std::to_string(imgIndex) + ".png";
        std::string thermalImagePath = thermalimageDirectory + std::to_string(imgIndex) + ".png";

        cv::Mat colorImage = cv::imread(colorImagePath, cv::IMREAD_COLOR);
        cv::Mat thermalImage = cv::imread(thermalImagePath, cv::IMREAD_GRAYSCALE);
        if (colorImage.empty() || thermalImage.empty())
        {
            std::cerr << "Failed to load image pair " << imgIndex << std::endl;
            continue;
        }

        std::vector<cv::Point2f> cornersColor, cornersThermal;
        bool foundColor = cv::findChessboardCorners(colorImage, patternSize, cornersColor);
        bool foundThermal = cv::findChessboardCorners(thermalImage, patternSize, cornersThermal);
        if (!foundColor || !foundThermal)
        {
            std::cerr << "Chessboard corners not found in image pair " << imgIndex << std::endl;
            continue;
        }

        cv::cornerSubPix(colorImage, cornersColor, cv::Size(7, 7), cv::Size(-1, -1),
                            cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.1));
        cv::cornerSubPix(thermalImage, cornersThermal, cv::Size(3, 3), cv::Size(-1, -1),
                            cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.1));
        cornersColorVec.push_back(cornersColor);
        cornersThermalVec.push_back(cornersThermal);
        objectPointsVec.push_back(objectPoints);
    }

    if (cornersColorVec.empty() || cornersThermalVec.empty())
    {
        std::cerr << "No valid chessboard corners detected in the images!" << std::endl;
        return -1;
    }

    cv::Mat R, T, E, F;
    double reprojectionError = cv::stereoCalibrate(objectPointsVec, cornersThermalVec, cornersColorVec,
                        cameraMatrixThermal, distCoeffsThermal,
                        cameraMatrixRealSense, distCoeffsRealSense,
                        cv::Size(), R, T, E, F,
                        cv::CALIB_FIX_INTRINSIC);

    std::cout << "Rotation Matrix:\n" << R << std::endl;
    std::cout << "Translation Vector:\n" << T << std::endl;

    std::string extfile = imageDirectory + "/../extrinsic.xml";
    cv::FileStorage fsOut(extfile, cv::FileStorage::WRITE);
    fsOut << "R" << R;
    fsOut << "T" << T;
    fsOut << "E" << E;
    fsOut << "F" << F;
    fsOut.release();
    std::cout << "Extrinsic parameters and stereo calibration outputs saved to extrinsic.xml" << std::endl;
    std::cout << "Reprojection Error: " << reprojectionError << std::endl;

    return 0;
}
