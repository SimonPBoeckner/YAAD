#pragma once
#include <opencv2/opencv.hpp>
#include <poseEstimator.hpp>
#include "dataTypes.hpp"

class PoseEstimator {
public:
    PoseEstimator();
    virtual ~PoseEstimator();

    virtual FiducialPoseObject SolveFiducialPose(zarray_t* detections);

protected:
    float fid_size;

    std::vector<cv::Point3d> object_points;
    std::vector<cv::Point2d> frame_points;

    cv::Mat cameraMatrix = (cv::Mat_<double>(3,3) <<
        600, 0, 320,
        0, 600, 240,
        0, 0, 1
    );

    cv::Mat distCoeffs = cv::Mat::zeros(5, 1, CV_64F);
    std::vector<cv::Mat> rvecs;
    std::vector<cv::Mat> tvecs;
    std::vector<double> reprojErrors;
};

class SquareTargetPoseEstimator: public PoseEstimator {
public:
    FiducialPoseObject SolveFiducialPose(zarray_t* detections) override;
};