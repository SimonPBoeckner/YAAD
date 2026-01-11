#include "cameraPoseEstimator.hpp"
#include "frc/geometry/Pose3d.h"
#include "frc/geometry/Translation3d.h"
#include "frc/geometry/Rotation3d.h"
#include "frc/geometry/Quaternion.h"
#include <nlohmann/json.hpp>
#include <iostream>
#include <vector>
#include <fstream>
#include "coordinateChanger.hpp"
#include "opencv2/opencv.hpp"
using json = nlohmann::json;

CameraPoseEstimator::CameraPoseEstimator() {
    std::ifstream file("config.json");
    if (!file.is_open()) {
        std::cerr << "Failed to open config.json\n";
        return;
    }

    file >> CameraPoseEstimator::jason;
}

CameraPoseEstimator::~CameraPoseEstimator() {}
void CameraPoseEstimator::SolveCameraPose(zarray_t* detections) {}

void MultiTagCameraPoseEstimator::SolveCameraPose(zarray_t* detections) {
    // if (field_layou == nullptr) return nothing

    if (detections == nullptr || zarray_size(detections) == 0) {
        return;
    }

    MultiTagCameraPoseEstimator::fid_size = 0.162f; // in meters

    for (int i = 0; i < zarray_size(detections); i++) {
        frc::Pose3d tag_pose;
        apriltag_detection_t* det;
        zarray_get(detections, i, &det);

        for (int j = 0; j < MultiTagCameraPoseEstimator::jason["tags"].size(); j++) {
            if (MultiTagCameraPoseEstimator::jason["tags"][j]["id"] == det->id) {
                tag_pose = frc::Pose3d(
                    frc::Translation3d(units::meter_t(MultiTagCameraPoseEstimator::jason["tags"][j]["pose"]["x"]), 
                                       units::meter_t(MultiTagCameraPoseEstimator::jason["tags"][j]["pose"]["y"]), 
                                       units::meter_t(MultiTagCameraPoseEstimator::jason["tags"][j]["pose"]["z"])),
                    frc::Rotation3d(
                        frc::Quaternion(
                            MultiTagCameraPoseEstimator::jason["tags"][j]["pose"]["qw"], 
                            MultiTagCameraPoseEstimator::jason["tags"][j]["pose"]["qx"], 
                            MultiTagCameraPoseEstimator::jason["tags"][j]["pose"]["qy"], 
                            MultiTagCameraPoseEstimator::jason["tags"][j]["pose"]["qz"]
                        )
                    )
                );
                break;
            }

            if (tag_pose == frc::Pose3d()) {
                
                frc::Pose3d corner_0 = tag_pose + frc::Transform3d(
                    frc::Translation3d(units::meter_t(fid_size / 2), units::meter_t(-fid_size / 2), 0_m),
                    frc::Rotation3d()
                );
                frc::Pose3d corner_1 = tag_pose + frc::Transform3d(
                    frc::Translation3d(units::meter_t(-fid_size / 2), units::meter_t(-fid_size / 2), 0_m),
                    frc::Rotation3d()
                );
                frc::Pose3d corner_2 = tag_pose + frc::Transform3d(
                    frc::Translation3d(units::meter_t(-fid_size / 2), units::meter_t(fid_size / 2), 0_m),
                    frc::Rotation3d()
                );
                frc::Pose3d corner_3 = tag_pose + frc::Transform3d(
                    frc::Translation3d(units::meter_t(fid_size / 2), units::meter_t(fid_size / 2), 0_m),
                    frc::Rotation3d()
                );
                object_points.push_back(
                    cv::Point3d(
                        WPILibTranslationToOpenCV(corner_0.Translation())[0],
                        WPILibTranslationToOpenCV(corner_0.Translation())[1],
                        WPILibTranslationToOpenCV(corner_0.Translation())[2]
                    )
                );
                object_points.push_back(
                    cv::Point3d(
                        WPILibTranslationToOpenCV(corner_1.Translation())[0],
                        WPILibTranslationToOpenCV(corner_1.Translation())[1],
                        WPILibTranslationToOpenCV(corner_1.Translation())[2]
                    )
                );
                object_points.push_back(
                    cv::Point3d(
                        WPILibTranslationToOpenCV(corner_2.Translation())[0],
                        WPILibTranslationToOpenCV(corner_2.Translation())[1],
                        WPILibTranslationToOpenCV(corner_2.Translation())[2]
                    )
                );
                object_points.push_back(
                    cv::Point3d(
                        WPILibTranslationToOpenCV(corner_3.Translation())[0],
                        WPILibTranslationToOpenCV(corner_3.Translation())[1],
                        WPILibTranslationToOpenCV(corner_3.Translation())[2]
                    )
                );

                frame_points.push_back(cv::Point2d(det->p[0][0], det->p[0][1]));
                frame_points.push_back(cv::Point2d(det->p[1][0], det->p[1][1]));
                frame_points.push_back(cv::Point2d(det->p[2][0], det->p[2][1]));
                frame_points.push_back(cv::Point2d(det->p[3][0], det->p[3][1]));

                tag_ids.push_back(det->id);
                tag_poses.push_back(tag_pose);
            }
        }

        if (tag_ids.size() == 1) {
            object_points.push_back(cv::Point3d(-fid_size / 2.0, fid_size / 2.0, 0.0));
            object_points.push_back(cv::Point3d(fid_size / 2.0, fid_size / 2.0, 0.0));
            object_points.push_back(cv::Point3d(fid_size / 2.0, -fid_size / 2.0, 0.0));
            object_points.push_back(cv::Point3d(-fid_size / 2.0, -fid_size / 2.0, 0.0));

            std::vector<cv::Mat> rvecs, tvecs;
            std::vector<double> reprojErrors;

            try {
                cv::solvePnPGeneric(
                    object_points,
                    frame_points,
                    cameraMatrix,
                    distCoeffs,
                    rvecs,
                    tvecs,
                    false,
                    cv::SOLVEPNP_IPPE_SQUARE,
                    cv::noArray(),
                    cv::noArray(),
                    reprojErrors
                );
            }
            catch (const cv::Exception& e) {
                std::cerr << "OpenCV Exception: " << e.what() << std::endl;
            }

            frc::Pose3d field_to_tag_pose = tag_poses[0];
            frc::Pose3d camera_to_tag_pose_0 = OpenCVPoseToWpiLib(tvecs[0], rvecs[0]);
            frc::Pose3d camera_to_tag_pose_1 = OpenCVPoseToWpiLib(tvecs[1], rvecs[1]);
            frc::Transform3d camera_to_tag_0 = {camera_to_tag_pose_0.Translation(), camera_to_tag_pose_0.Rotation()};
            frc::Transform3d camera_to_tag_1 = {camera_to_tag_pose_1.Translation(), camera_to_tag_pose_1.Rotation()};
            frc::Pose3d field_to_camera_0 = field_to_tag_pose.TransformBy(
                camera_to_tag_0.Inverse()
            );
            frc::Pose3d field_to_field_1 = field_to_tag_pose.TransformBy(
                camera_to_tag_1.Inverse()
            );
            frc::Pose3d field_to_camera_pose_0 = {
                field_to_camera_0.Translation(),
                field_to_camera_0.Rotation()
            };
            frc::Pose3d field_to_camera_pose_1 = {
                field_to_field_1.Translation(),
                field_to_field_1.Rotation()
            };
        }
    }
}