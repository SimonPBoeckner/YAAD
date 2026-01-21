#include "coordinateChanger.hpp"
#include <cmath>
#include <Eigen/Dense>

namespace CoordinateConverter {

std::optional<frc::Pose3d> OpenCVPoseToWPILib(const cv::Mat& tvec, const cv::Mat& rvec) {
    if (tvec.empty() || rvec.empty() || 
        tvec.rows != 3 || tvec.cols != 1 ||
        rvec.rows != 3 || rvec.cols != 1) {
        return std::nullopt;
    }
    
    try {
        // Extract translation (OpenCV -> WPILib coordinate conversion)
        double tx = tvec.at<double>(2, 0);
        double ty = -tvec.at<double>(0, 0);
        double tz = -tvec.at<double>(1, 0);

        // Extract rotation (OpenCV -> WPILib coordinate conversion)
        double rx = rvec.at<double>(2, 0);
        double ry = -rvec.at<double>(0, 0);
        double rz = -rvec.at<double>(1, 0);

        double angle = std::sqrt(
            rvec.at<double>(0, 0) * rvec.at<double>(0, 0) +
            rvec.at<double>(1, 0) * rvec.at<double>(1, 0) +
            rvec.at<double>(2, 0) * rvec.at<double>(2, 0)
        );

        return frc::Pose3d(
            frc::Translation3d{units::meter_t(tx), units::meter_t(ty), units::meter_t(tz)},
            frc::Rotation3d{Eigen::Vector3d(rx, ry, rz), units::radian_t(angle)}
        );
    } catch (const std::exception& e) {
        return std::nullopt;
    }
}

std::vector<double> WPILibTranslationToOpenCV(const frc::Translation3d& translation) {
    return {
        -translation.Y().value(),
        -translation.Z().value(),
        translation.X().value()
    };
}

} // namespace CoordinateConverter