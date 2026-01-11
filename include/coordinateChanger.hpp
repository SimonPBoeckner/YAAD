#pragma once
#include <cmath>
#include <vector>

#include <opencv2/opencv.hpp>

#include "frc/geometry/Pose3d.h"
#include "frc/geometry/Translation3d.h"
#include "frc/geometry/Rotation3d.h"
#include "frc/geometry/Quaternion.h"

frc::Pose3d OpenCVPoseToWpiLib(const cv::Mat& tvec, const cv::Mat& rvec);
std::vector<double> WPILibTranslationToOpenCV(const frc::Translation3d& translation);