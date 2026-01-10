#pragma once
#include <apriltag/apriltag.h>
#include <opencv2/opencv.hpp>

class CameraPoseEstimator {
public:
    CameraPoseEstimator();
    virtual ~CameraPoseEstimator();

    virtual void SolveCameraPose(zarray_t* detections);

protected:
    float fid_size;
    
};