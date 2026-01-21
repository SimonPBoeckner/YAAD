#pragma once
#include "multiCameraManager.hpp"
#include "kalmanFilter.hpp"
#include <memory>
#include <unordered_map>

struct FusedPoseResult {
    frc::Pose3d pose;
    Eigen::Vector3d velocity;
    double confidence;
    int numCamerasUsed;
    std::chrono::system_clock::time_point timestamp;
    std::vector<std::string> contributingCameras;
};

class CameraFusion {
public:
    CameraFusion();
    
    // Process detections from all cameras and produce fused estimate
    FusedPoseResult FuseDetections(
        const std::vector<CameraDetectionResult>& detections
    );
    
    // Set fusion parameters
    void SetMaxMeasurementAge(double seconds);
    void SetMinCamerasRequired(int count);
    void SetProcessNoise(double noise);
    
    // Reset filter
    void Reset();

private:
    PoseKalmanFilter kalmanFilter;
    std::chrono::system_clock::time_point lastUpdateTime;
    
    double maxMeasurementAge; // seconds
    int minCamerasRequired;
    
    // Weight detections based on reprojection error
    double CalculateWeight(const CameraPoseObject& poseData) const;
    
    // Fuse multiple pose estimates using weighted average
    std::optional<frc::Pose3d> WeightedAveragePoses(
        const std::vector<std::pair<frc::Pose3d, double>>& posesWithWeights
    ) const;
    
    // Convert camera pose to robot/world pose using camera mount
    frc::Pose3d TransformCameraPoseToWorld(
        const frc::Pose3d& cameraPose,
        const frc::Pose3d& cameraMountPose
    ) const;
};