#pragma once
#include <apriltag/apriltag.h>
#include <apriltag/tag36h11.h>
#include <opencv2/opencv.hpp>
#include "dataTypes.hpp"

struct DetectorConfig {
    float quad_decimate = 1.0f;
    float quad_sigma = 0.0f;
    int nthreads = 1;
    bool debug = false;
    bool refine_edges = true;
};

class FiducialDetector {
public:
    explicit FiducialDetector(const DetectorConfig& config = DetectorConfig{});
    ~FiducialDetector();
    
    // Delete copy/move to prevent issues with apriltag pointers
    FiducialDetector(const FiducialDetector&) = delete;
    FiducialDetector& operator=(const FiducialDetector&) = delete;
    FiducialDetector(FiducialDetector&&) = delete;
    FiducialDetector& operator=(FiducialDetector&&) = delete;

    // Returns ownership of detections to caller via unique_ptr
    ZArrayPtr DetectFiducials(const cv::Mat& frame);

private:
    void ApplyConfig(const DetectorConfig& config);
    
    apriltag_family_t* tf = nullptr;
    apriltag_detector_t* td = nullptr;
    DetectorConfig currentConfig;
};