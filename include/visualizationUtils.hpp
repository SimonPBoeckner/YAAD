#pragma once
#include <opencv2/opencv.hpp>
#include <apriltag/apriltag.h>
#include "dataTypes.hpp"

class VisualizationUtils {
public:
    // Draw AprilTag detections on frame
    static void DrawDetections(cv::Mat& frame, zarray_t* detections);
    
    // Draw pose information overlay
    static void DrawPoseOverlay(
        cv::Mat& frame, 
        const CameraPoseObject& poseData,
        const std::string& cameraName
    );
    
    // Draw FPS counter
    static void DrawFPS(cv::Mat& frame, double fps);
    
    // Draw confidence bar
    static void DrawConfidence(cv::Mat& frame, double confidence);

private:
    static cv::Scalar GetColorForConfidence(double confidence);
};