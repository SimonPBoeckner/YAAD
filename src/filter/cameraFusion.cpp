#include "cameraFusion.hpp"
#include "logger.hpp"
#include <cmath>

CameraFusion::CameraFusion()
    : maxMeasurementAge(0.5)
    , minCamerasRequired(1) {
    lastUpdateTime = std::chrono::system_clock::now();
}

FusedPoseResult CameraFusion::FuseDetections(
    const std::vector<CameraDetectionResult>& detections) {
    
    auto currentTime = std::chrono::system_clock::now();
    auto dt = std::chrono::duration<double>(currentTime - lastUpdateTime).count();
    
    // Prediction step
    if (kalmanFilter.IsInitialized()) {
        kalmanFilter.Predict(dt);
    }
    
    // Collect valid measurements with weights
    std::vector<std::pair<frc::Pose3d, double>> posesWithWeights;
    std::vector<std::string> contributingCameras;
    
    for (const auto& detection : detections) {
        // Check if detection is recent enough
        auto age = std::chrono::duration<double>(
            currentTime - detection.timestamp
        ).count();
        
        if (age > maxMeasurementAge) {
            continue;
        }
        
        // Check if detection has valid pose data
        if (!detection.poseData.isValid()) {
            continue;
        }
        
        // Calculate weight based on reprojection error
        double weight = CalculateWeight(detection.poseData);
        
        if (weight > 0.0) {
            posesWithWeights.push_back({detection.poseData.pose_0, weight});
            contributingCameras.push_back(detection.cameraName);
        }
    }
    
    FusedPoseResult result;
    result.timestamp = currentTime;
    result.numCamerasUsed = posesWithWeights.size();
    result.contributingCameras = contributingCameras;
    
    // Check if we have enough measurements
    if (posesWithWeights.size() >= static_cast<size_t>(minCamerasRequired)) {
        // Fuse poses using weighted average
        auto fusedPoseOpt = WeightedAveragePoses(posesWithWeights);
        
        if (fusedPoseOpt) {
            // Update Kalman filter with fused measurement
            double avgWeight = 0.0;
            for (const auto& pw : posesWithWeights) {
                avgWeight += pw.second;
            }
            avgWeight /= posesWithWeights.size();
            
            // Convert weight to measurement noise (higher weight = lower noise)
            double measurementNoise = 1.0 / (avgWeight + 0.01);
            
            kalmanFilter.Update(*fusedPoseOpt, measurementNoise);
            result.confidence = avgWeight;
        } else {
            result.confidence = 0.0;
        }
    } else if (kalmanFilter.IsInitialized()) {
        // Not enough measurements, use prediction only
        result.confidence = 0.3; // Low confidence for prediction-only
    } else {
        // No measurements and not initialized
        result.confidence = 0.0;
    }
    
    // Get final estimate from Kalman filter
    if (kalmanFilter.IsInitialized()) {
        result.pose = kalmanFilter.GetEstimatedPose();
        result.velocity = kalmanFilter.GetVelocity();
    } else {
        result.pose = frc::Pose3d();
        result.velocity = Eigen::Vector3d::Zero();
        result.confidence = 0.0;
    }
    
    lastUpdateTime = currentTime;
    return result;
}

void CameraFusion::SetMaxMeasurementAge(double seconds) {
    maxMeasurementAge = seconds;
}

void CameraFusion::SetMinCamerasRequired(int count) {
    minCamerasRequired = count;
}

void CameraFusion::SetProcessNoise(double noise) {
    kalmanFilter.SetProcessNoise(noise);
}

void CameraFusion::Reset() {
    kalmanFilter.Reset();
}

double CameraFusion::CalculateWeight(const CameraPoseObject& poseData) const {
    if (!poseData.isValid()) {
        return 0.0;
    }
    
    // Weight based on reprojection error (lower error = higher weight)
    // Also consider number of tags detected
    double errorWeight = 1.0 / (poseData.error_0 + 0.1);
    double tagWeight = std::sqrt(static_cast<double>(poseData.tag_ids.size()));
    
    return errorWeight * tagWeight;
}

std::optional<frc::Pose3d> CameraFusion::WeightedAveragePoses(
    const std::vector<std::pair<frc::Pose3d, double>>& posesWithWeights) const {
    
    if (posesWithWeights.empty()) {
        return std::nullopt;
    }
    
    // Weighted average of positions
    double totalWeight = 0.0;
    Eigen::Vector3d avgPosition(0, 0, 0);
    
    for (const auto& [pose, weight] : posesWithWeights) {
        avgPosition += Eigen::Vector3d(
            pose.Translation().X().value(),
            pose.Translation().Y().value(),
            pose.Translation().Z().value()
        ) * weight;
        totalWeight += weight;
    }
    
    avgPosition /= totalWeight;
    
    // For orientation, we use the pose with highest weight
    // (Averaging quaternions is more complex and this is a simpler approach)
    double maxWeight = 0.0;
    frc::Rotation3d bestRotation;
    
    for (const auto& [pose, weight] : posesWithWeights) {
        if (weight > maxWeight) {
            maxWeight = weight;
            bestRotation = pose.Rotation();
        }
    }
    
    return frc::Pose3d(
        frc::Translation3d(
            units::meter_t(avgPosition.x()),
            units::meter_t(avgPosition.y()),
            units::meter_t(avgPosition.z())
        ),
        bestRotation
    );
}

frc::Pose3d CameraFusion::TransformCameraPoseToWorld(
    const frc::Pose3d& cameraPose,
    const frc::Pose3d& cameraMountPose) const {
    
    // Transform from camera coordinate frame to world frame
    frc::Transform3d cameraToWorld(
        cameraMountPose.Translation(),
        cameraMountPose.Rotation()
    );
    
    return cameraPose.TransformBy(cameraToWorld);
}