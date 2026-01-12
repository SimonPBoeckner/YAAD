#pragma once
#include <opencv2/opencv.hpp>
#include <apriltag/apriltag.h>
#include "frc/geometry/Pose3d.h"
#include <vector>
#include <optional>

template<typename T>
using Opt = std::optional<T>;

struct CameraPoseObject {
    std::vector<int> tag_ids;

    frc::Pose3d pose_0;
    double error_0;

    Opt<frc::Pose3d> pose_1;  // optional
    Opt<double> error_1;      // optional
};
