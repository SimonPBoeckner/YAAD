#pragma once
#include <apriltag/apriltag.h>
#include <opencv2/opencv.hpp>
#include "frc/geometry/Pose3d.h"
#include <vector>
#include "dataTypes.hpp"
#include <nlohmann/json.hpp>

class CameraPoseEstimator {
public:
    CameraPoseEstimator();
    virtual ~CameraPoseEstimator();

    virtual CameraPoseObject SolveCameraPose(zarray_t* detections);

protected:
    nlohmann::json jason;
    bool found = true;
    float fid_size;
    std::vector<cv::Point3d> object_points;
    std::vector<cv::Point2d> frame_points;
    std::vector<int> tag_ids;
    std::vector<frc::Pose3d> tag_poses;

    frc::Pose3d corner_0;

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

class MultiTagCameraPoseEstimator: public CameraPoseEstimator {
public:
    CameraPoseObject SolveCameraPose(zarray_t* detections) override;
};