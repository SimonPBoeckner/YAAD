#include "tagAngleCalculator.hpp"
#include <opencv2/opencv.hpp>
#include <iostream>

TagAngleCalculator::TagAngleCalculator(const CameraConfig& config)
    : config(config), poseEstimator(config) {}

TagAngleObject TagAngleCalculator::CalculateTagAngle(zarray_t* detections) {
    return TagAngleObject{};
}

void TagAngleCalculator::SetCameraConfig(const CameraConfig& newConfig) {
    config = newConfig;
    poseEstimator.SetCameraConfig(newConfig);
}

CameraMatrixTagAngleCalculator::CameraMatrixTagAngleCalculator(const CameraConfig& config)
    : TagAngleCalculator(config) {}

TagAngleObject CameraMatrixTagAngleCalculator::CalculateTagAngle(zarray_t* detections) {
    if (!detections || zarray_size(detections) == 0) {
        return TagAngleObject{};
    }

    std::vector<cv::Point2f> distortedPoints;
    
    // Collect all corner points from all detections
    for (int i = 0; i < zarray_size(detections); i++) {
        apriltag_detection_t* det;
        zarray_get(detections, i, &det);
        if (!det) continue;
        
        for (int j = 0; j < 4; j++) {
            distortedPoints.emplace_back(
                static_cast<float>(det->p[j][0]),
                static_cast<float>(det->p[j][1])
            );
        }
    }

    if (distortedPoints.empty()) {
        return TagAngleObject{};
    }

    // Undistort points
    std::vector<cv::Point2f> imagePoints;
    try {
        cv::undistortPoints(
            distortedPoints,
            imagePoints,
            config.cameraMatrix,
            config.distCoeffs,
            cv::noArray(),
            config.cameraMatrix
        );
    } catch (const cv::Exception& e) {
        std::cerr << "Failed to undistort points: " << e.what() << std::endl;
        return TagAngleObject{};
    }

    // Convert camera matrix to Eigen for inverse calculation
    Eigen::Matrix3d K = config.toEigen();
    Eigen::Matrix3d K_inv = K.inverse();

    // Calculate angular coordinates for first tag's corners
    Eigen::Matrix<double, 4, 2> corners;
    for (int i = 0; i < std::min(4, static_cast<int>(imagePoints.size())); i++) {
        Eigen::Vector3d img_pt(imagePoints[i].x, imagePoints[i].y, 1.0);
        Eigen::Vector3d vec = K_inv * img_pt;
        
        corners(i, 0) = std::atan(vec(0));
        corners(i, 1) = std::atan(vec(1));
    }

    // Get pose estimation for distance
    FiducialPoseObject fiducialPose = poseEstimator.SolveFiducialPose(detections);
    if (!fiducialPose.isValid()) {
        return TagAngleObject{};
    }

    double distance = (fiducialPose.error0 < fiducialPose.error1)
        ? fiducialPose.pose0.Translation().Norm().value()
        : fiducialPose.pose1.Translation().Norm().value();

    return TagAngleObject{
        .tag_id = fiducialPose.tag_id,
        .corners = corners,
        .distance = distance
    };
}