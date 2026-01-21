#include "kalmanFilter.hpp"
#include "frc/geometry/Rotation3d.h"

PoseKalmanFilter::PoseKalmanFilter() : initialized(false) {
    state.setZero();
    covariance.setIdentity();
    covariance *= 1.0; // Initial uncertainty
    
    processNoise.setIdentity();
    processNoise *= 0.01; // Default process noise
}

void PoseKalmanFilter::Initialize(const frc::Pose3d& initialPose) {
    PoseToState(initialPose, state);
    initialized = true;
}

void PoseKalmanFilter::Predict(double dt) {
    if (!initialized) return;
    
    // State transition matrix (constant velocity model)
    Eigen::Matrix<double, STATE_SIZE, STATE_SIZE> F;
    F.setIdentity();
    
    // Position updates from velocity
    F(0, 3) = dt; // x += vx * dt
    F(1, 4) = dt; // y += vy * dt
    F(2, 5) = dt; // z += vz * dt
    
    // Orientation updates from angular velocity
    F(6, 9) = dt;  // roll += w_roll * dt
    F(7, 10) = dt; // pitch += w_pitch * dt
    F(8, 11) = dt; // yaw += w_yaw * dt
    
    // Predict state
    state = F * state;
    
    // Predict covariance
    covariance = F * covariance * F.transpose() + processNoise;
}

void PoseKalmanFilter::Update(const frc::Pose3d& measurement, double measurementNoise) {
    if (!initialized) {
        Initialize(measurement);
        return;
    }
    
    // Measurement matrix (we measure position and orientation)
    Eigen::Matrix<double, 6, STATE_SIZE> H;
    H.setZero();
    H(0, 0) = 1.0; // x
    H(1, 1) = 1.0; // y
    H(2, 2) = 1.0; // z
    H(3, 6) = 1.0; // roll
    H(4, 7) = 1.0; // pitch
    H(5, 8) = 1.0; // yaw
    
    // Measurement noise covariance
    Eigen::Matrix<double, 6, 6> R;
    R.setIdentity();
    R *= measurementNoise;
    
    // Convert measurement to vector
    Eigen::Matrix<double, STATE_SIZE, 1> measurementState;
    PoseToState(measurement, measurementState);
    
    Eigen::Matrix<double, 6, 1> z;
    z << measurementState(0), measurementState(1), measurementState(2),
         measurementState(6), measurementState(7), measurementState(8);
    
    // Innovation
    Eigen::Matrix<double, 6, 1> y = z - H * state;
    
    // Normalize angles in innovation
    for (int i = 3; i < 6; i++) {
        while (y(i) > M_PI) y(i) -= 2 * M_PI;
        while (y(i) < -M_PI) y(i) += 2 * M_PI;
    }
    
    // Innovation covariance
    Eigen::Matrix<double, 6, 6> S = H * covariance * H.transpose() + R;
    
    // Kalman gain
    Eigen::Matrix<double, STATE_SIZE, 6> K = covariance * H.transpose() * S.inverse();
    
    // Update state
    state = state + K * y;
    
    // Update covariance
    Eigen::Matrix<double, STATE_SIZE, STATE_SIZE> I;
    I.setIdentity();
    covariance = (I - K * H) * covariance;
}

frc::Pose3d PoseKalmanFilter::GetEstimatedPose() const {
    return StateToPose(state);
}

Eigen::Vector3d PoseKalmanFilter::GetVelocity() const {
    return Eigen::Vector3d(state(3), state(4), state(5));
}

void PoseKalmanFilter::SetProcessNoise(double noise) {
    processNoise.setIdentity();
    processNoise *= noise;
}

void PoseKalmanFilter::Reset() {
    initialized = false;
    state.setZero();
    covariance.setIdentity();
}

void PoseKalmanFilter::PoseToState(
    const frc::Pose3d& pose, 
    Eigen::Matrix<double, STATE_SIZE, 1>& stateVec) {
    
    // Position
    stateVec(0) = pose.Translation().X().value();
    stateVec(1) = pose.Translation().Y().value();
    stateVec(2) = pose.Translation().Z().value();
    
    // Velocities (keep current if updating)
    if (!initialized) {
        stateVec(3) = 0;
        stateVec(4) = 0;
        stateVec(5) = 0;
    }
    
    // Orientation (Euler angles)
    auto rotation = pose.Rotation();
    stateVec(6) = rotation.X().value(); // roll
    stateVec(7) = rotation.Y().value(); // pitch
    stateVec(8) = rotation.Z().value(); // yaw
    
    // Angular velocities (keep current if updating)
    if (!initialized) {
        stateVec(9) = 0;
        stateVec(10) = 0;
        stateVec(11) = 0;
    }
}

frc::Pose3d PoseKalmanFilter::StateToPose(
    const Eigen::Matrix<double, STATE_SIZE, 1>& stateVec) const {
    
    return frc::Pose3d(
        frc::Translation3d(
            units::meter_t(stateVec(0)),
            units::meter_t(stateVec(1)),
            units::meter_t(stateVec(2))
        ),
        frc::Rotation3d(
            units::radian_t(stateVec(6)),
            units::radian_t(stateVec(7)),
            units::radian_t(stateVec(8))
        )
    );
}