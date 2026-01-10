#include "cameraPoseEstimator.hpp"

CameraPoseEstimator::CameraPoseEstimator() {}

CameraPoseEstimator::~CameraPoseEstimator() {}

void CameraPoseEstimator::SolveCameraPose(zarray_t* detections) {
    // if (field_layou == nullptr) return nothing

    if (detections == nullptr || zarray_size(detections) == 0) {
        return;
    }
}