#include <cmath>
#include <vector>

#include <opencv2/opencv.hpp>
#include "apriltag/apriltag.h"
#include "frc/geometry/Pose3d.h"
#include "frc/geometry/Translation3d.h"
#include "frc/geometry/Rotation3d.h"
#include "frc/geometry/Quaternion.h"
#include <Eigen/Dense>

using namespace frc;

frc::Pose3d OpenCVPoseToWPILib(const std::vector<cv::Mat>& tvec, const std::vector<cv::Mat>& rvec) {
    // Extract values
    double tx = tvec[0].at<double>(2, 0);
    double ty = -tvec[0].at<double>(0, 0);
    double tz = -tvec[0].at<double>(1, 0);

    double rx = rvec[0].at<double>(2, 0);
    double ry = -rvec[0].at<double>(0, 0);
    double rz = -rvec[0].at<double>(1, 0);

    double angle = std::sqrt(
        std::pow(rvec[0].at<double>(0, 0), 2) +
        std::pow(rvec[0].at<double>(1, 0), 2) +
        std::pow(rvec[0].at<double>(2, 0), 2)
    );

    return frc::Pose3d(
        frc::Translation3d{units::meter_t(tx), units::meter_t(ty), units::meter_t(tz)},
        frc::Rotation3d{Eigen::Vector3d(rx, ry, rz), units::radian_t(angle)}
    );
}

std::vector<double> WPILibTranslationToOpenCV(const frc::Translation3d& translation) {
    return {
        double(-translation.Y()),
        double(-translation.Z()),
        double(translation.X())
    };
}
