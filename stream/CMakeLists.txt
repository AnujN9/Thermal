cmake_minimum_required(VERSION 3.10)
project(lepton_project)

# Set the C++ standard
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find Packages to run the programs
find_package(realsense2 REQUIRED)
find_package(OpenCV REQUIRED)

# Include directories for OpenCV and the current directory
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${CMAKE_SOURCE_DIR}/include)

# Make directories for the images to be stored
file(MAKE_DIRECTORY ${CMAKE_BINARY_DIR}/thermal_images)
file(MAKE_DIRECTORY ${CMAKE_BINARY_DIR}/images)

# Define the executable
add_executable(lepton lepton.cpp Palettes.cpp)
add_executable(depth_saver depthimage.cpp Palettes.cpp)

# Link the libraries
target_link_libraries(lepton ${OpenCV_LIBS})
target_link_libraries(depth_saver ${realsense2_LIBRARY} ${OpenCV_LIBS})
