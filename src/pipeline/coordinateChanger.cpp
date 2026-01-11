#include <cmath>
#include <vector>

#include <opencv2/opencv.hpp>

#include "frc/geometry/Pose3d.h"
#include "frc/geometry/Translation3d.h"
#include "frc/geometry/Rotation3d.h"
#include "frc/geometry/Quaternion.h"
#include <Eigen/Dense>

using namespace frc;

frc::Pose3d OpenCVPoseToWPILib(const cv::Mat& tvec, const cv::Mat& rvec) {
    // Extract values
    double tx = tvec.at<double>(2, 0);
    double ty = -tvec.at<double>(0, 0);
    double tz = -tvec.at<double>(1, 0);

    double rx = rvec.at<double>(2, 0);
    double ry = -rvec.at<double>(0, 0);
    double rz = -rvec.at<double>(1, 0);

    double angle = std::sqrt(
        std::pow(rvec.at<double>(0, 0), 2) +
        std::pow(rvec.at<double>(1, 0), 2) +
        std::pow(rvec.at<double>(2, 0), 2)
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
