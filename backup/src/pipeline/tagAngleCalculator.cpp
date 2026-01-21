#include "tagAngleCalculator.hpp"
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>
#include "poseEstimator.hpp"
#include <Eigen/Dense>
#include <dataTypes.hpp>
#include <poseEstimator.hpp>

using namespace cv;

TagAngleCalculator::TagAngleCalculator() {}
TagAngleCalculator::~TagAngleCalculator() {}

TagAngleObject TagAngleCalculator::CalculateTagAngle(zarray_t* detections) {
    return TagAngleObject{};
}

TagAngleObject CameraMatrixTagAngleCalculator::CalculateTagAngle(zarray_t* detections) {
    Eigen::MatrixXd A;
    int k = 0;

    for (int i = 0; i < zarray_size(detections); i++) {
        apriltag_detection_t* det;
        zarray_get(detections, i, &det);
        for (int j = 0; j < 4; j++) {
        A(0, k) = double(det->p[j][0]); 
        A(1, k) = double(det->p[j][1]);
        k++;
        }
    };

    undistortPoints(
        distortedPoints,
        imagePoints,
        cameraMat,
        distCoeffs,
        cv::noArray(),
        cameraMat
    );

    Eigen::Matrix3d K = Eigen::Map<Eigen::Matrix<double,3,3,Eigen::RowMajor>>(
    cameraMat.ptr<double>());
    Eigen::Matrix3d k_inv = K.inverse();

    for (size_t index = 0; index < imagePoints.size(); ++index) {

        const auto& corner = imagePoints[index];

        // Homogeneous image point
        Eigen::Vector3d img_pt(
            corner.x,
            corner.y,
            1.0
        );

        // Back-project into camera space
        Eigen::Vector3d vec = k_inv * img_pt;

        // Store angular coordinates
        corners(index, 0) = std::atan(vec(0));
        corners(index, 1) = std::atan(vec(1));
    }

    FiducialPoseObject fiducialPose = poseEstimator.SolveFiducialPose(detections);
    float distance = 0;
    if (fiducialPose.error0 < fiducialPose.error1) {
        distance = fiducialPose.pose0.Translation().Norm().to<double>();
        return TagAngleObject{fiducialPose.tag_id, corners, distance};
    }
    else {
        distance = fiducialPose.pose1.Translation().Norm().to<double>();
        return TagAngleObject{fiducialPose.tag_id, corners, distance};
    }
    return TagAngleObject{};
}