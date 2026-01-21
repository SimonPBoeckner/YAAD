#pragma once
#include "dataTypes.hpp"
#include <Eigen/Dense>
#include <poseEstimator.hpp>

class TagAngleCalculator {
public:
    TagAngleCalculator();
    virtual ~TagAngleCalculator();

    virtual TagAngleObject CalculateTagAngle(zarray_t* detections);
protected:
    Eigen::Matrix3d cameraMatrix = (Eigen::Matrix3d() <<
        600, 0,   320,
        0,   600, 240,
        0,   0,   1
    ).finished();

    SquareTargetPoseEstimator poseEstimator;

    cv::Mat cameraMat = (cv::Mat_<double>(3,3) << 600, 0, 320, 0, 600, 240, 0, 0, 1 );

    cv::Mat distCoeffs = cv::Mat::zeros(5, 1, CV_64F);

    std::vector<cv::Point2f> imagePoints;
    std::vector<cv::Point2f> distortedPoints;
    Eigen::Matrix<double, 4, 2> corners;
};

class CameraMatrixTagAngleCalculator : public TagAngleCalculator {
public:
    TagAngleObject CalculateTagAngle(zarray_t* detections) override;
};