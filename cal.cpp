#include <opencv2/opencv.hpp>
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include <vector>
#include <iostream>

cv::Mat gray, edge;
int detectSquareMesh(cv::Mat& image);
int thresh_1 = 0, thresh_2 = 100;

void Trackbar_threshold(int, void*)
{
    thresh_1 = cv::getTrackbarPos("Threshold 1", "Canny Edge");
    thresh_2 = cv::getTrackbarPos("Threshold 2", "Canny Edge");
    cv::Canny(gray, edge, thresh_1, thresh_2);
    cv::imshow("Canny Edge", edge);
}

int main()
{
    std::vector<cv::Mat> calibrationImages;
    for (int i = 1; i <= 19; ++i)
    {
        std::string filename = "images/thermal_image_" + std::to_string(i) + ".png";
        cv::Mat image = cv::imread(filename, cv::IMREAD_COLOR);
        if (image.empty())
        {
            std::cerr << "Could not open or find the image " << filename << std::endl;
            return -1;
        }
        calibrationImages.push_back(image);
    }

    cv::namedWindow("Canny Edge", cv::WINDOW_AUTOSIZE);
    cv::createTrackbar("Threshold 1", "Canny Edge", nullptr, 255, 0);
    cv::createTrackbar("Threshold 2", "Canny Edge", nullptr, 255, 0);
        

    for (auto &img : calibrationImages)
    {
        cv::imshow("Image", img);
        cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);
        cv::imshow("Gray Image", gray);
        cv::bitwise_not(gray, gray);
        cv::imshow("Inverted image", gray);
        cv::waitKey(0);

        cv::setTrackbarPos("Threshold 1", "Canny Edge", 0);
        cv::setTrackbarPos("Threshold 2", "Canny Edge", 100);
        while (true)
        {
            Trackbar_threshold(0,0);
            if (cv::waitKey(30) >= 0)
            {
                break;
            }
        }
        detectSquareMesh(edge);
    }

    return 0;
}

int detectSquareMesh(cv::Mat& image)
{
    std::vector<cv::Vec2f> lines;
    cv::HoughLines(image, lines, 1, CV_PI/180, 150, 0, 0);

    for (size_t i = 0; i < lines.size(); i++)
    {
        float rho = lines[i][0], theta = lines[i][1];
        cv::Point pt1, pt2;
        double a = cos(theta), b = sin(theta);
        double x0 = a*rho, y0 = b*rho;
        pt1.x = cvRound(x0 + 1000*(-b));
        pt1.y = cvRound(y0 + 1000*(a));
        pt2.x = cvRound(x0 - 1000*(-b));
        pt2.y = cvRound(y0 - 1000*(a));
        cv::line(image, pt1, pt2, cv::Scalar(0,0,255), 3, cv::LINE_AA);
    }
    cv::imshow("Lines", image);
    cv::waitKey(0);
    return 0;
}
