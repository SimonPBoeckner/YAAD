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
                object_points.push_back(WPILibTranslationToOpenCV(corner_0.Translation()));
                object_points.push_back(WPILibTranslationToOpenCV(corner_1.Translation()));
                object_points.push_back(WPILibTranslationToOpenCV(corner_2.Translation()));
                object_points.push_back(WPILibTranslationToOpenCV(corner_3.Translation()));

                frame_points
            }
        }
    }
}