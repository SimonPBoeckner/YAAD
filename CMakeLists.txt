cmake_minimum_required(VERSION 4.2.1)
project(MyProject LANGUAGES CXX )

find_package( OpenCV REQUIRED )
find_package( apriltag REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )
include_directories( ${CMAKE_SOURCE_DIR}/include )  # Adjust path if capture.hpp is elsewhere
add_executable( MyProgram src/main.cpp src/pipeline/capture.cpp src/pipeline/fiducialDetector.cpp src/pipeline/cameraPoseEstimator.cpp )
target_link_libraries( MyProgram PRIVATE ${OpenCV_LIBS} )
target_link_libraries( MyProgram PRIVATE apriltag::apriltag )
