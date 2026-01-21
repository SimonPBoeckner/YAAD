#pragma once
#include "dataTypes.hpp"
#include "poseEstimator.hpp"
#include <Eigen/Dense>

class TagAngleCalculator {
public:
    explicit TagAngleCalculator(const CameraConfig& config = CameraConfig::Default());
    virtual ~TagAngleCalculator() = default;

    virtual TagAngleObject CalculateTagAngle(zarray_t* detections);
    
    void SetCameraConfig(const CameraConfig& config);

protected:
    CameraConfig config;
    SquareTargetPoseEstimator poseEstimator;
};

class CameraMatrixTagAngleCalculator : public TagAngleCalculator {
public:
    explicit CameraMatrixTagAngleCalculator(const CameraConfig& config = CameraConfig::Default());
    TagAngleObject CalculateTagAngle(zarray_t* detections) override;
};