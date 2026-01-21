#include "poseEstimator.hpp"
#include "coordinateChanger.hpp"
#include <iostream>

PoseEstimator::PoseEstimator(const CameraConfig& config) 
    : config(config) {}

FiducialPoseObject PoseEstimator::SolveFiducialPose(zarray_t* detections) {
    return FiducialPoseObject{};
}

void PoseEstimator::SetCameraConfig(const CameraConfig& newConfig) {
    config = newConfig;
}

SquareTargetPoseEstimator::SquareTargetPoseEstimator(const CameraConfig& config)
    : PoseEstimator(config) {}

FiducialPoseObject SquareTargetPoseEstimator::SolveFiducialPose(zarray_t* detections) {
    if (!detections || zarray_size(detections) == 0) {
        return FiducialPoseObject{};
    }
    
    apriltag_detection_t* det;
    zarray_get(detections, 0, &det);
    if (!det) {
        return FiducialPoseObject{};
    }

    const double halfSize = config.tagSize / 2.0;
    
    // Define 3D points in tag coordinate system
    std::vector<cv::Point3d> objectPoints = {
        cv::Point3d(-halfSize, halfSize, 0.0),
        cv::Point3d(halfSize, halfSize, 0.0),
        cv::Point3d(halfSize, -halfSize, 0.0),
        cv::Point3d(-halfSize, -halfSize, 0.0)
    };

    // Extract 2D image points
    std::vector<cv::Point2d> framePoints = {
        cv::Point2d(det->p[0][0], det->p[0][1]),
        cv::Point2d(det->p[1][0], det->p[1][1]),
        cv::Point2d(det->p[2][0], det->p[2][1]),
        cv::Point2d(det->p[3][0], det->p[3][1])
    };

    std::vector<cv::Mat> rvecs, tvecs;
    std::vector<double> reprojErrors;

    try {
        cv::solvePnPGeneric(
            objectPoints,
            framePoints,
            config.cameraMatrix,
            config.distCoeffs,
            rvecs,
            tvecs,
            false,
            cv::SOLVEPNP_IPPE_SQUARE,
            cv::noArray(),
            cv::noArray(),
            reprojErrors
        );

        if (rvecs.size() >= 2 && tvecs.size() >= 2) {
            auto pose0 = CoordinateConverter::OpenCVPoseToWPILib(tvecs[0], rvecs[0]);
            auto pose1 = CoordinateConverter::OpenCVPoseToWPILib(tvecs[1], rvecs[1]);
            
            if (pose0 && pose1) {
                return FiducialPoseObject{
                    .tag_id = static_cast<int>(det->id),
                    .pose0 = *pose0,
                    .error0 = reprojErrors.size() > 0 ? reprojErrors[0] : 0.0,
                    .pose1 = *pose1,
                    .error1 = reprojErrors.size() > 1 ? reprojErrors[1] : 0.0
                };
            }
        }
    } catch (const cv::Exception& e) {
        std::cerr << "OpenCV Exception in SolveFiducialPose: " << e.what() << std::endl;
    }
    
    return FiducialPoseObject{};
}