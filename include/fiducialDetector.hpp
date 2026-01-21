#pragma once
#include <apriltag/apriltag.h>
#include <apriltag/tag36h11.h>
#include <opencv2/opencv.hpp>

// extern "C" {
// #include "apriltag.h"
// #include "tag36h11.h"
// }

class FiducialDetector {
public:
    FiducialDetector();
    virtual ~FiducialDetector();

    virtual zarray_t *DetectFiducials(cv::Mat& frame);

    protected:
    apriltag_family_t *tf;
    apriltag_detector_t *td;
    cv::Mat frame;
};