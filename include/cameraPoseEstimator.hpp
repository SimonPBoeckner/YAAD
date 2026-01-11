#pragma once
#include <apriltag/apriltag.h>
#include <opencv2/opencv.hpp>
#include "json.hpp"

class CameraPoseEstimator {
public:
    CameraPoseEstimator();
    virtual ~CameraPoseEstimator();

    virtual void SolveCameraPose(zarray_t* detections);

protected:
    json jason;

    float fid_size;
    zarray_t object_points;
    zarray_t frame_points;
    zarray_t tag_ids;
    zarray_t tag_poses;
};