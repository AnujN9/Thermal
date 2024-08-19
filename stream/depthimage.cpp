#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <ctime>
#include <stdint.h>
#include <thread>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <arpa/inet.h>
#include <unistd.h>
#include <Palettes.h>

#define PACKET_SIZE 164
#define PACKET_SIZE_UINT16 (PACKET_SIZE/2)
#define PACKETS_PER_FRAME 60
#define FRAME_SIZE_UINT16 (PACKET_SIZE_UINT16*PACKETS_PER_FRAME)
#define FPS 27;
#define PORT 8080

int main(int argc, char * argv[])
try
{
    int sockfd;
    struct sockaddr_in servaddr, cliaddr;

    int typeColormap = 3; // colormap_ironblack
	int typeLepton = 3; // Lepton 3.x
	unsigned int spiSpeed = 20 * 1000 * 1000; // SPI bus speed 20MHz
	uint16_t loglevel = 0;
	const int *selectedColormap = colormap_ironblack;
	int selectedColormapSize = get_size_colormap_ironblack();
	bool autoRangeMin = false;
	bool autoRangeMax = false;
	uint16_t rangeMin = 27300; // Minimum temperture range (temp / 100 - 273) celsius
	uint16_t rangeMax = 33500; // Maximum temperture range
	int myImageWidth = 160;
	int myImageHeight = 120;
	int img_cnt = 0;
	bool first = true;

	uint8_t result[PACKET_SIZE*PACKETS_PER_FRAME];
	uint8_t shelf[4][PACKET_SIZE*PACKETS_PER_FRAME];

	uint16_t minValue = rangeMin;
	uint16_t maxValue = rangeMax;
	float diff = maxValue - minValue;
	float scale = 255/diff;
	uint16_t n_zero_value_drop_frame = 0;

    cv::Mat image(myImageHeight, myImageWidth, CV_8UC3);
	cv::Mat gray(myImageHeight, myImageWidth, CV_8UC1);
    cv::namedWindow("Thermal Image", cv::WINDOW_AUTOSIZE);

    // Create socket
    if ((sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0)
	{
        std::cerr << "Socket creation failed" << std::endl;
        return -1;
    }

    memset(&servaddr, 0, sizeof(servaddr));
    memset(&cliaddr, 0, sizeof(cliaddr));
    
    // Set up server address
    servaddr.sin_family = AF_INET;
    servaddr.sin_port = htons(PORT);
    servaddr.sin_addr.s_addr = INADDR_ANY;

    // Bind the socket
    if (bind(sockfd, (const struct sockaddr *)&servaddr, sizeof(servaddr)) < 0)
	{
        std::cerr << "Bind failed" << std::endl;
        close(sockfd);
        return -1;
    }

    socklen_t len = sizeof(cliaddr);

    // Declare RealSense pipeline, encapsulating the actual device and sensors
    rs2::pipeline pipe;

    // Start streaming with default settings
    pipe.start();

    int frame_counter = 0;

    while (true)
    {
		// Wait for the next set of frames from the camera
        rs2::frameset frames = pipe.wait_for_frames();

        // Get color frame
        rs2::video_frame color_frame = frames.get_color_frame();

        for (int i = 0; i < 4; ++i)
		{
            ssize_t received_bytes = recvfrom(sockfd, shelf[i], sizeof(shelf[i]), 0, (struct sockaddr *)&cliaddr, &len);
            if (received_bytes < 0)
			{
                std::cerr << "Receive failed" << std::endl;
                close(sockfd);
                return -1;
            }
			if (first && i == 1)
			{
				first = false;
				break;
			}
        }
        if (autoRangeMin || autoRangeMax)
		{
			if (autoRangeMin)
			{
				maxValue = 65535;
			}
			if (autoRangeMax)
			{
				maxValue = 0;
			}
			for(int iSegment = 1; iSegment <= 4; iSegment++)
			{
				for(int i=0;i<FRAME_SIZE_UINT16;i++)
				{
					//skip the first 2 uint16_t's of every packet, they're 4 header bytes
					if(i % PACKET_SIZE_UINT16 < 2)
					{
						continue;
					}

					//flip the MSB and LSB at the last second
					uint16_t value = (shelf[iSegment - 1][i*2] << 8) + shelf[iSegment - 1][i*2+1];
					if (value == 0)
					{
						// Why this value is 0?
						continue;
					}
					if (autoRangeMax && (value > maxValue))
					{
						maxValue = value;
					}
					if (autoRangeMin && (value < minValue))
					{
						minValue = value;
					}
				}
			}
			diff = maxValue - minValue;
			scale = 255/diff;
		}
		int row, column;
		uint16_t value;
		uint16_t valueFrameBuffer;
		for(int iSegment = 1; iSegment <= 4; iSegment++)
		{
			int ofsRow = 30 * (iSegment - 1);
			for(int i=0;i<FRAME_SIZE_UINT16;i++)
			{
				//skip the first 2 uint16_t's of every packet, they're 4 header bytes
				if(i % PACKET_SIZE_UINT16 < 2)
				{
					continue;
				}

				//flip the MSB and LSB at the last second
				valueFrameBuffer = (shelf[iSegment - 1][i*2] << 8) + shelf[iSegment - 1][i*2+1];
				if (valueFrameBuffer == 0)
				{
					// Why this value is 0?
					n_zero_value_drop_frame++;
					break;
				}

				//
				if (!autoRangeMin && (valueFrameBuffer <= minValue))
				{
					value = minValue;
				}
				else if (!autoRangeMax && (valueFrameBuffer > maxValue))
				{
					value = maxValue;
				}
				else
				{
					value = ((valueFrameBuffer - minValue) * scale);
				}
				int ofs_r = 3 * value + 0; if (selectedColormapSize <= ofs_r) ofs_r = selectedColormapSize - 1;
				int ofs_g = 3 * value + 1; if (selectedColormapSize <= ofs_g) ofs_g = selectedColormapSize - 1;
				int ofs_b = 3 * value + 2; if (selectedColormapSize <= ofs_b) ofs_b = selectedColormapSize - 1;
				cv::Vec3b color(selectedColormap[ofs_b], selectedColormap[ofs_g], selectedColormap[ofs_r]);
                column = (i % PACKET_SIZE_UINT16) - 2 + (myImageWidth / 2) * ((i % (PACKET_SIZE_UINT16 * 2)) / PACKET_SIZE_UINT16);
                row = i / PACKET_SIZE_UINT16 / 2 + ofsRow;
                if (row < myImageHeight && column < myImageWidth)
				{
                    image.at<cv::Vec3b>(row, column) = color;
					gray.at<uint8_t>(row, column) = value;
                }
                if ((column == 80) && (row == 60))
				{
                    double temp = (valueFrameBuffer / 100) - 273;
                    std::cout << "Temp at center " << temp << std::endl;
                }
			}
		}
		if (n_zero_value_drop_frame != 0)
		{
			n_zero_value_drop_frame = 0;
		}

        // Get the dimensions of the frame
        const int width = color_frame.get_width();
        const int height = color_frame.get_height();

        // Create OpenCV matrix from color image (assume 8-bit RGB image)
        cv::Mat color_image(cv::Size(width, height), CV_8UC3, (void*)color_frame.get_data(), cv::Mat::AUTO_STEP);

        // Convert the image from RGB to BGR format (since OpenCV uses BGR by default)
        cv::cvtColor(color_image, color_image, cv::COLOR_RGB2BGR);

        // Display the color image in a window
        cv::imshow("Color Image", color_image);

        // Wait for keypress for 1ms, 'c' to capture and 'q' to quit
        char key = cv::waitKey(1);

        cv::imshow("Thermal Image", image);

        if (key == 'c' || key == 'C')
        {
            // Save the color image to disk when 'c' is pressed
            std::string filename = "color_image_" + std::to_string(frame_counter) + ".png";
            cv::imwrite(filename, color_image);

            std::cout << "Saved color image: " << filename << std::endl;
            frame_counter++;

            img_cnt++;
            filename = "thermal_image_" + std::to_string(img_cnt) + ".png";
			cv::imwrite(filename, image);
			filename = "thermal_grayimage_" + std::to_string(img_cnt) + ".png";
			cv::imwrite(filename, gray);
			std::cout << "Image saved" << std::endl;
        }
        else if (key == 'q' || key == 'Q')
        {
            // Exit the loop if 'q' is pressed
            break;
        }
    }
    // Close the socket
    close(sockfd);
    return 0;
}
catch (const rs2::error & e)
{
    // Capture RealSense exceptions
    std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
    return EXIT_FAILURE;
}
catch (const std::exception & e)
{
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
}
