#pragma once
#include <opencv2/opencv.hpp>
#include "dataTypes.hpp"

class PoseEstimator {
public:
    explicit PoseEstimator(const CameraConfig& config = CameraConfig::Default());
    virtual ~PoseEstimator() = default;

    virtual FiducialPoseObject SolveFiducialPose(zarray_t* detections);
    
    void SetCameraConfig(const CameraConfig& config);
    const CameraConfig& GetCameraConfig() const { return config; }

protected:
    CameraConfig config;
};

class SquareTargetPoseEstimator : public PoseEstimator {
public:
    explicit SquareTargetPoseEstimator(const CameraConfig& config = CameraConfig::Default());
    FiducialPoseObject SolveFiducialPose(zarray_t* detections) override;
};