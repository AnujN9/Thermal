cmake_minimum_required(VERSION 3.10)
project(Calibration)

# Set the C++ standard
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find packages
find_package(OpenCV REQUIRED)
find_package(realsense2 REQUIRED)

# Include directories for OpenCV
include_directories(${OpenCV_INCLUDE_DIRS})

# Define the Image directory for calibration
file(MAKE_DIRECTORY ${CMAKE_SOURCE_DIR}/thermal_images)
file(MAKE_DIRECTORY ${CMAKE_SOURCE_DIR}/color_images)
add_definitions(-DTHERMAL_IMAGE_DIRECTORY="${CMAKE_SOURCE_DIR}/thermal_images")
add_definitions(-DIMAGE_DIRECTORY="${CMAKE_SOURCE_DIR}/color_images")

# Define the executable
add_executable(camera_calibration cal.cpp)
add_executable(verify_calibration verify.cpp)
add_executable(extrinsic extrinsic_cal.cpp)

# Link the libraries to executables
target_link_libraries(camera_calibration ${OpenCV_LIBS})
target_link_libraries(verify_calibration ${OpenCV_LIBS})
target_link_libraries(extrinsic ${OpenCV_LIBS} ${realsense2_LIBRARY})