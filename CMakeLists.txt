cmake_minimum_required(VERSION 4.2.1)
project(YAAD LANGUAGES CXX )
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# set(THREADS_PREFER_PTHREAD_FLAG TRUE) 

find_package( Threads REQUIRED )
find_package( OpenCV REQUIRED )
find_package( apriltag REQUIRED )
find_package(nlohmann_json REQUIRED)
find_package( wpilib REQUIRED )
find_package( Eigen3 REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )
include_directories( ${CMAKE_SOURCE_DIR}/include )
include_directories( ${CMAKE_SOURCE_DIR}/src/config )
add_executable( YAAD 
    src/main.cpp 
    src/pipeline/capture.cpp 
    src/pipeline/fiducialDetector.cpp 
    src/pipeline/cameraPoseEstimator.cpp 
    src/pipeline/coordinateChanger.cpp 
    src/pipeline/poseEstimator.cpp 
    src/pipeline/tagAngleCalculator.cpp 
    src/pipeline/multiCameraManager.cpp
    src/config/configManager.cpp
    src/util/logger.cpp
    src/util/webServer.cpp
    src/util/networkStreamer.cpp
    src/util/performanceMonitor.cpp
    src/util/logger.cpp
    src/filter/cameraFusion.cpp
    src/filter/kalmanFilter.cpp
    src/util/mjpegStreamer.cpp
    src/util/visualizationUtils.cpp
)
target_link_libraries( YAAD PRIVATE ${OpenCV_LIBS} )
target_link_libraries( YAAD PRIVATE apriltag )
target_link_libraries( YAAD PRIVATE cameraserver ntcore cscore wpiutil wpimath wpilibc hal )
target_link_libraries( YAAD PRIVATE nlohmann_json::nlohmann_json)
target_link_libraries( YAAD PRIVATE Eigen3::Eigen )
target_link_libraries( YAAD PRIVATE Threads::Threads )
