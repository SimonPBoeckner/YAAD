#pragma once
#include <Eigen/Dense>
#include "frc/geometry/Pose3d.h"
#include <chrono>

// 6-DOF Kalman filter for pose estimation (x, y, z, roll, pitch, yaw)
class PoseKalmanFilter {
public:
    PoseKalmanFilter();
    
    // Initialize filter with initial pose
    void Initialize(const frc::Pose3d& initialPose);
    
    // Predict step (motion model)
    void Predict(double dt);
    
    // Update step (measurement)
    void Update(const frc::Pose3d& measurement, double measurementNoise);
    
    // Get current estimated pose
    frc::Pose3d GetEstimatedPose() const;
    
    // Get current velocity estimate
    Eigen::Vector3d GetVelocity() const;
    
    // Set process noise (how much we trust the model)
    void SetProcessNoise(double noise);
    
    // Reset filter
    void Reset();
    
    bool IsInitialized() const { return initialized; }

private:
    // State vector: [x, y, z, vx, vy, vz, roll, pitch, yaw, w_roll, w_pitch, w_yaw]
    static const int STATE_SIZE = 12;
    
    Eigen::Matrix<double, STATE_SIZE, 1> state;
    Eigen::Matrix<double, STATE_SIZE, STATE_SIZE> covariance;
    Eigen::Matrix<double, STATE_SIZE, STATE_SIZE> processNoise;
    
    bool initialized;
    
    // Convert Pose3d to state vector
    void PoseToState(const frc::Pose3d& pose, Eigen::Matrix<double, STATE_SIZE, 1>& stateVec);
    
    // Convert state vector to Pose3d
    frc::Pose3d StateToPose(const Eigen::Matrix<double, STATE_SIZE, 1>& stateVec) const;
};