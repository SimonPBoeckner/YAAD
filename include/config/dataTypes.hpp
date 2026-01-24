#pragma once
#include <opencv2/opencv.hpp>
#include <apriltag/apriltag.h>
#include "frc/geometry/Pose3d.h"
#include <vector>
#include <optional>
#include <memory>
#include <Eigen/Dense>

template<typename T>
using Opt = std::optional<T>;

// RAII wrapper for zarray_t to prevent memory leaks
class ZArrayDeleter {
public:
    void operator()(zarray_t* arr) const {
        if (arr) {
            zarray_destroy(arr);
        }
    }
};

using ZArrayPtr = std::unique_ptr<zarray_t, ZArrayDeleter>;

struct CameraConfig {
    cv::Mat cameraMatrix;
    cv::Mat distCoeffs;
    double tagSize; // in meters
    
    static CameraConfig Default() {
        CameraConfig config;
        config.cameraMatrix = (cv::Mat_<double>(3,3) <<
            600, 0, 320,
            0, 600, 240,
            0, 0, 1
        );
        config.distCoeffs = cv::Mat::zeros(5, 1, CV_64F);
        config.tagSize = 0.162;
        return config;
    }
    
    Eigen::Matrix3d toEigen() const {
        Eigen::Matrix3d mat;
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                mat(i, j) = cameraMatrix.at<double>(i, j);
            }
        }
        return mat;
    }
};

struct CameraPoseObject {
    std::vector<int> tag_ids;
    frc::Pose3d pose_0;
    double error_0 = 0.0;
    Opt<frc::Pose3d> pose_1;
    Opt<double> error_1;
    
    bool isValid() const {
        return !tag_ids.empty();
    }
};

struct TagAngleObject {
    int tag_id = -1;
    Eigen::Matrix<double, 4, 2> corners;
    double distance = 0.0;
    
    bool isValid() const {
        return tag_id >= 0;
    }
};

struct FiducialPoseObject {
    int tag_id = -1;
    frc::Pose3d pose0;
    double error0 = 0.0;
    frc::Pose3d pose1;
    double error1 = 0.0;
    
    bool isValid() const {
        return tag_id >= 0;
    }
};