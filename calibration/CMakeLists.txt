cmake_minimum_required(VERSION 3.10)
project(verify_project)

# Set the C++ standard
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find OpenCV using pkg-config
find_package(PkgConfig REQUIRED)
pkg_check_modules(OpenCV REQUIRED opencv4)

# Include directories for OpenCV
include_directories(${OpenCV_INCLUDE_DIRS})

# Define the executable for dot.cpp
add_executable(dot_pattern dot.cpp)

# Define the executable for cal.cpp
add_executable(camera_calibration cal.cpp)

# Define the executable for verify_dot.cpp
add_executable(verify_dot_pattern verify_dot.cpp)

# Define the executable for chess_verify.cpp
add_executable(verify_calibration chess_verify.cpp)

# Link the OpenCV libraries to both executables
target_link_libraries(dot_pattern ${OpenCV_LIBRARIES})
target_link_libraries(camera_calibration ${OpenCV_LIBRARIES})
target_link_libraries(verify_dot_pattern ${OpenCV_LIBRARIES})
target_link_libraries(verify_calibration ${OpenCV_LIBRARIES})
