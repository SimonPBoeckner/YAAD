#pragma once
#include <opencv2/opencv.hpp>
#include "frc/geometry/Pose3d.h"
#include "frc/geometry/Translation3d.h"
#include "frc/geometry/Rotation3d.h"
#include <optional>
#include <vector>

namespace CoordinateConverter {
    // Convert OpenCV pose (tvec, rvec) to WPILib coordinate system
    std::optional<frc::Pose3d> OpenCVPoseToWPILib(const cv::Mat& tvec, const cv::Mat& rvec);
    
    // Convert WPILib translation to OpenCV coordinate system
    std::vector<double> WPILibTranslationToOpenCV(const frc::Translation3d& translation);
}