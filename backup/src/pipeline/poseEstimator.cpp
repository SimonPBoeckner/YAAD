#include "poseEstimator.hpp"
#include "coordinateChanger.hpp"

PoseEstimator::PoseEstimator() {}
PoseEstimator::~PoseEstimator() {}

FiducialPoseObject PoseEstimator::SolveFiducialPose(zarray_t* detections) {
    return FiducialPoseObject{};
}

FiducialPoseObject SquareTargetPoseEstimator::SolveFiducialPose(zarray_t* detections) {
    fid_size = 0.162f;
    apriltag_detection_t* det;
    zarray_get(detections, 0, &det);

    object_points.push_back(cv::Point3d(-fid_size / 2.0, fid_size / 2.0, 0.0));
    object_points.push_back(cv::Point3d(fid_size / 2.0, fid_size / 2.0, 0.0));
    object_points.push_back(cv::Point3d(fid_size / 2.0, -fid_size / 2.0, 0.0));
    object_points.push_back(cv::Point3d(-fid_size / 2.0, -fid_size / 2.0, 0.0));

    frame_points.push_back(cv::Point2d(det->p[0][0], det->p[0][1]));
    frame_points.push_back(cv::Point2d(det->p[1][0], det->p[1][1]));
    frame_points.push_back(cv::Point2d(det->p[2][0], det->p[2][1]));
    frame_points.push_back(cv::Point2d(det->p[3][0], det->p[3][1]));


    try {
        cv::solvePnPGeneric(
                    object_points,
                    frame_points,
                    cameraMatrix,
                    distCoeffs,
                    rvecs,
                    tvecs,
                    false,
                    cv::SOLVEPNP_IPPE_SQUARE,
                    cv::noArray(),
                    cv::noArray(),
                    reprojErrors
        );
    }
    catch (const cv::Exception& e) {
        std::cerr << "OpenCV Exception: " << e.what() << std::endl;
    }

    try {
        return FiducialPoseObject{
            .tag_id = int(det->id),
            .pose0 = OpenCVPoseToWPILib(tvecs[0], rvecs[0]),
            .error0 = reprojErrors[0],
            .pose1 = OpenCVPoseToWPILib(tvecs[1], rvecs[1]),
            .error1 = reprojErrors[1]
        };
    }
    catch (const cv::Exception& e) {
        std::cerr << "OpenCV Exception: " << e.what() << std::endl;
    }
    return FiducialPoseObject{};
}