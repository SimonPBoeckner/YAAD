#include "fiducialDetector.hpp"
#include <iostream>

FiducialDetector::FiducialDetector(const DetectorConfig& config) 
    : currentConfig(config) {
    tf = tag36h11_create();
    if (!tf) {
        throw std::runtime_error("Failed to create tag family");
    }
    
    td = apriltag_detector_create();
    if (!td) {
        tag36h11_destroy(tf);
        throw std::runtime_error("Failed to create apriltag detector");
    }
    
    apriltag_detector_add_family(td, tf);
    ApplyConfig(config);
}

FiducialDetector::~FiducialDetector() {
    if (td) {
        apriltag_detector_destroy(td);
    }
    if (tf) {
        tag36h11_destroy(tf);
    }
}

void FiducialDetector::ApplyConfig(const DetectorConfig& config) {
    if (!td) return;
    
    td->quad_decimate = config.quad_decimate;
    td->quad_sigma = config.quad_sigma;
    td->nthreads = config.nthreads;
    td->debug = config.debug ? 1 : 0;
    td->refine_edges = config.refine_edges ? 1 : 0;
}

ZArrayPtr FiducialDetector::DetectFiducials(const cv::Mat& frame) {
    if (!td || frame.empty()) {
        return nullptr;
    }
    
    cv::Mat gray;
    if (frame.channels() == 3) {
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
    } else if (frame.channels() == 4) {
        cv::cvtColor(frame, gray, cv::COLOR_BGRA2GRAY);
    } else {
        gray = frame.clone();
    }

    image_u8_t im = {
        .width = gray.cols,
        .height = gray.rows,
        .stride = gray.cols,
        .buf = gray.data
    };

    zarray_t* detections = apriltag_detector_detect(td, &im);
    return ZArrayPtr(detections);
}