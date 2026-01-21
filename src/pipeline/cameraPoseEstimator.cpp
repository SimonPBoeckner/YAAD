#include "cameraPoseEstimator.hpp"
#include "coordinateChanger.hpp"
#include <fstream>
#include <iostream>

// FieldLayout Implementation
FieldLayout::FieldLayout(const std::string& jsonPath) {
    LoadFromJson(jsonPath);
}

void FieldLayout::LoadFromJson(const std::string& jsonPath) {
    std::ifstream file(jsonPath);
    if (!file.is_open()) {
        throw std::runtime_error("Failed to open field layout file: " + jsonPath);
    }

    file >> jsonData;
    
    if (!jsonData.contains("tags")) {
        throw std::runtime_error("Invalid field layout JSON: missing 'tags' field");
    }

    for (const auto& tag : jsonData["tags"]) {
        if (!tag.contains("ID") || !tag.contains("pose")) {
            continue;
        }
        
        int id = tag["ID"];
        const auto& pose = tag["pose"];
        
        frc::Pose3d tagPose(
            frc::Translation3d(
                units::meter_t(pose["translation"]["x"].get<double>()),
                units::meter_t(pose["translation"]["y"].get<double>()),
                units::meter_t(pose["translation"]["z"].get<double>())
            ),
            frc::Rotation3d(
                frc::Quaternion(
                    pose["rotation"]["quaternion"]["W"].get<double>(),
                    pose["rotation"]["quaternion"]["X"].get<double>(),
                    pose["rotation"]["quaternion"]["Y"].get<double>(),
                    pose["rotation"]["quaternion"]["Z"].get<double>()
                )
            )
        );
        
        tagPoses[id] = tagPose;
    }
}

std::optional<frc::Pose3d> FieldLayout::GetTagPose(int tagId) const {
    auto it = tagPoses.find(tagId);
    if (it != tagPoses.end()) {
        return it->second;
    }
    return std::nullopt;
}

bool FieldLayout::HasTag(int tagId) const {
    return tagPoses.find(tagId) != tagPoses.end();
}

// CameraPoseEstimator Implementation
CameraPoseEstimator::CameraPoseEstimator(const CameraConfig& config)
    : config(config) {}

CameraPoseObject CameraPoseEstimator::SolveCameraPose(zarray_t* detections) {
    return CameraPoseObject{};
}

void CameraPoseEstimator::SetCameraConfig(const CameraConfig& newConfig) {
    config = newConfig;
}

void CameraPoseEstimator::SetFieldLayout(std::shared_ptr<FieldLayout> layout) {
    fieldLayout = layout;
}

// MultiTagCameraPoseEstimator Implementation
MultiTagCameraPoseEstimator::MultiTagCameraPoseEstimator(
    const CameraConfig& config,
    std::shared_ptr<FieldLayout> layout)
    : CameraPoseEstimator(config) {
    fieldLayout = layout;
}

CameraPoseObject MultiTagCameraPoseEstimator::SolveCameraPose(zarray_t* detections) {
    if (!detections || zarray_size(detections) == 0) {
        return CameraPoseObject{};
    }
    
    if (!fieldLayout) {
        std::cerr << "No field layout set\n";
        return CameraPoseObject{};
    }

    std::vector<cv::Point3d> objectPoints;
    std::vector<cv::Point2d> framePoints;
    std::vector<int> tagIds;
    std::vector<frc::Pose3d> tagPoses;

    const double halfSize = config.tagSize / 2.0;

    // Collect all detected tags with known field positions
    for (int i = 0; i < zarray_size(detections); i++) {
        apriltag_detection_t* det;
        zarray_get(detections, i, &det);
        if (!det) continue;

        auto tagPose = fieldLayout->GetTagPose(det->id);
        if (!tagPose) {
            std::cerr << "Tag ID " << det->id << " not found in field layout\n";
            continue;
        }

        // Calculate corner positions in field coordinates
        std::vector<frc::Translation3d> corners = {
            (*tagPose + frc::Transform3d(
                frc::Translation3d(units::meter_t(halfSize), units::meter_t(-halfSize), 0_m),
                frc::Rotation3d()
            )).Translation(),
            (*tagPose + frc::Transform3d(
                frc::Translation3d(units::meter_t(-halfSize), units::meter_t(-halfSize), 0_m),
                frc::Rotation3d()
            )).Translation(),
            (*tagPose + frc::Transform3d(
                frc::Translation3d(units::meter_t(-halfSize), units::meter_t(halfSize), 0_m),
                frc::Rotation3d()
            )).Translation(),
            (*tagPose + frc::Transform3d(
                frc::Translation3d(units::meter_t(halfSize), units::meter_t(halfSize), 0_m),
                frc::Rotation3d()
            )).Translation()
        };

        // Convert to OpenCV coordinates and add to points
        for (const auto& corner : corners) {
            auto cv_coords = CoordinateConverter::WPILibTranslationToOpenCV(corner);
            objectPoints.emplace_back(cv_coords[0], cv_coords[1], cv_coords[2]);
        }

        // Add image points
        for (int j = 0; j < 4; j++) {
            framePoints.emplace_back(det->p[j][0], det->p[j][1]);
        }

        tagIds.push_back(det->id);
        tagPoses.push_back(*tagPose);
    }

    if (tagIds.empty()) {
        return CameraPoseObject{};
    }

    // Single tag: use IPPE_SQUARE for two solutions
    if (tagIds.size() == 1) {
        apriltag_detection_t* det;
        zarray_get(detections, 0, &det);
        return SolveSingleTag(det, tagPoses[0]);
    }

    // Multiple tags: use SQPNP
    return SolveMultiTag(objectPoints, framePoints, tagIds);
}

CameraPoseObject MultiTagCameraPoseEstimator::SolveSingleTag(
    apriltag_detection_t* det, 
    const frc::Pose3d& tagPose) {
    
    const double halfSize = config.tagSize / 2.0;
    
    std::vector<cv::Point3d> objectPoints = {
        cv::Point3d(-halfSize, halfSize, 0.0),
        cv::Point3d(halfSize, halfSize, 0.0),
        cv::Point3d(halfSize, -halfSize, 0.0),
        cv::Point3d(-halfSize, -halfSize, 0.0)
    };

    std::vector<cv::Point2d> framePoints = {
        cv::Point2d(det->p[0][0], det->p[0][1]),
        cv::Point2d(det->p[1][0], det->p[1][1]),
        cv::Point2d(det->p[2][0], det->p[2][1]),
        cv::Point2d(det->p[3][0], det->p[3][1])
    };

    std::vector<cv::Mat> rvecs, tvecs;
    std::vector<double> reprojErrors;

    try {
        cv::solvePnPGeneric(
            objectPoints,
            framePoints,
            config.cameraMatrix,
            config.distCoeffs,
            rvecs,
            tvecs,
            false,
            cv::SOLVEPNP_IPPE_SQUARE,
            cv::noArray(),
            cv::noArray(),
            reprojErrors
        );

        if (rvecs.size() >= 2 && tvecs.size() >= 2) {
            auto cameraPose0 = CoordinateConverter::OpenCVPoseToWPILib(tvecs[0], rvecs[0]);
            auto cameraPose1 = CoordinateConverter::OpenCVPoseToWPILib(tvecs[1], rvecs[1]);
            
            if (cameraPose0 && cameraPose1) {
                frc::Transform3d cameraToTag0(cameraPose0->Translation(), cameraPose0->Rotation());
                frc::Transform3d cameraToTag1(cameraPose1->Translation(), cameraPose1->Rotation());
                
                frc::Pose3d fieldToCamera0 = tagPose.TransformBy(cameraToTag0.Inverse());
                frc::Pose3d fieldToCamera1 = tagPose.TransformBy(cameraToTag1.Inverse());

                return CameraPoseObject{
                    .tag_ids = {static_cast<int>(det->id)},
                    .pose_0 = fieldToCamera0,
                    .error_0 = reprojErrors[0],
                    .pose_1 = fieldToCamera1,
                    .error_1 = reprojErrors[1]
                };
            }
        }
    } catch (const cv::Exception& e) {
        std::cerr << "OpenCV Exception in SolveSingleTag: " << e.what() << std::endl;
    }

    return CameraPoseObject{};
}

CameraPoseObject MultiTagCameraPoseEstimator::SolveMultiTag(
    const std::vector<cv::Point3d>& objectPoints,
    const std::vector<cv::Point2d>& framePoints,
    const std::vector<int>& tagIds) {
    
    std::vector<cv::Mat> rvecs, tvecs;
    std::vector<double> reprojErrors;

    try {
        cv::solvePnPGeneric(
            objectPoints,
            framePoints,
            config.cameraMatrix,
            config.distCoeffs,
            rvecs,
            tvecs,
            false,
            cv::SOLVEPNP_SQPNP,
            cv::noArray(),
            cv::noArray(),
            reprojErrors
        );

        if (!rvecs.empty() && !tvecs.empty()) {
            auto cameraToField = CoordinateConverter::OpenCVPoseToWPILib(tvecs[0], rvecs[0]);
            
            if (cameraToField) {
                frc::Transform3d transform(cameraToField->Translation(), cameraToField->Rotation());
                frc::Pose3d fieldToCamera(
                    transform.Inverse().Translation(),
                    transform.Inverse().Rotation()
                );

                return CameraPoseObject{
                    .tag_ids = tagIds,
                    .pose_0 = fieldToCamera,
                    .error_0 = reprojErrors.empty() ? 0.0 : reprojErrors[0]
                };
            }
        }
    } catch (const cv::Exception& e) {
        std::cerr << "OpenCV Exception in SolveMultiTag: " << e.what() << std::endl;
    }

    return CameraPoseObject{};
}