#include "fiducialDetector.hpp"
#include <apriltag/apriltag.h>
#include <apriltag/tag36h11.h>

// extern "C" {
// #include "apriltag.h"
// #include "tag36h11.h"
// }

FiducialDetector::FiducialDetector() {
    tf = tag36h11_create();
    td = apriltag_detector_create();
    apriltag_detector_add_family(td, tf);
    td->quad_decimate = 1.0;
    td->quad_sigma = 0.0;
    td->nthreads = 1;
    td->debug = 0;
    td->refine_edges = 1;
}

FiducialDetector::~FiducialDetector() {
    apriltag_detector_destroy(td);
    tag36h11_destroy(tf);
}

zarray_t* FiducialDetector::DetectFiducials(cv::Mat& frame) {
    this->frame = frame;
    cv::Mat gray;
    if (frame.channels() == 3) {
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
    } else if (frame.channels() == 4) {
        cv::cvtColor(frame, gray, cv::COLOR_BGRA2GRAY);
    } else {
        gray = frame;
    }

    image_u8_t im = { .width = gray.cols,
                      .height = gray.rows,
                      .stride = gray.cols,
                      .buf = gray.data
                    };

    zarray_t *detections = apriltag_detector_detect(td, &im);
    return detections;
}