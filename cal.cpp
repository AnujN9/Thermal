#include <opencv2/opencv.hpp>
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include <vector>
#include <iostream>

int detectSquareMesh(cv::Mat& image);

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

    cv::Size patternSize(4, 4);
    std::vector<std::vector<cv::Point2f>> imagePoints;
    for (auto& img : calibrationImages)
    {
        cv::Mat gray;
        cv::imshow("Image", img);
        cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);
        cv::imshow("Gray Image", gray);
        cv::bitwise_not(gray, gray);
        cv::imshow("Inverted image", gray);
        cv::waitKey(0);
        cv::Mat edge;
        cv::Canny(gray, edge, 127, 255, 3);
        cv::imshow("Edges", edge);
        cv::waitKey(0);
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
