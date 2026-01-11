#include "cameraPoseEstimator.hpp"
#include "frc/geometry/Pose3d.h"
#include "frc/geometry/Translation3d.h"
#include "frc/geometry/Rotation3d.h"
#include "json.hpp"
#include <iostream>
#include <fstream>
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

void CameraPoseEstimator::SolveCameraPose(zarray_t* detections) {
    // if (field_layou == nullptr) return nothing

    if (detections == nullptr || zarray_size(detections) == 0) {
        return;
    }

    CameraPoseEstimator::fid_size = 0.162f; // in meters
    CameraPoseEstimator::object_points = zarray_create(sizeof());
    CameraPoseEstimator::frame_points = zarray_create(sizeof());
    CameraPoseEstimator::tag_ids = zarray_create(sizeof());
    CameraPoseEstimator::tag_poses = zarray_create(sizeof());

    for (int i = 0; i < zarray_size(detections); i++) {
        frc::Pose3d tag_pose;
        apriltag_detection_t* det;
        zarray_get(detections, i, &det);
        for (int j = 0; j < CameraPoseEstimator::jason["tags"].size(); j++) {
            if (CameraPoseEstimator::jason["tags"][j]["id"] == det->id) {
                float x = CameraPoseEstimator::jason["tags"][j]["pose"]["x"];
                float y = CameraPoseEstimator::jason["tags"][j]["pose"]["y"];
                float z = CameraPoseEstimator::jason["tags"][j]["pose"]["z"];
                float roll = CameraPoseEstimator::jason["tags"][j]["pose"]["roll"];
                float pitch = CameraPoseEstimator::jason["tags"][j]["pose"]["pitch"];
                float yaw = CameraPoseEstimator::jason["tags"][j]["pose"]["yaw"];
                tag_pose = frc::Pose3d(
                    frc::Translation3d(x, y, z),
                    frc::Rotation3d(roll, pitch, yaw)
                );
                break;
            }
        }
    }
}