#pragma once
#include <apriltag/apriltag.h>
#include <opencv2/opencv.hpp>
#include "frc/geometry/Pose3d.h"
#include <nlohmann/json.hpp>
#include <string>
#include <unordered_map>
#include "dataTypes.hpp"

class FieldLayout {
public:
    explicit FieldLayout(const std::string& jsonPath);
    
    std::optional<frc::Pose3d> GetTagPose(int tagId) const;
    bool HasTag(int tagId) const;

private:
    void LoadFromJson(const std::string& jsonPath);
    
    std::unordered_map<int, frc::Pose3d> tagPoses;
    nlohmann::json jsonData;
};

class CameraPoseEstimator {
public:
    explicit CameraPoseEstimator(const CameraConfig& config = CameraConfig::Default());
    virtual ~CameraPoseEstimator() = default;

    virtual CameraPoseObject SolveCameraPose(zarray_t* detections);
    
    void SetCameraConfig(const CameraConfig& config);
    void SetFieldLayout(std::shared_ptr<FieldLayout> layout);

protected:
    CameraConfig config;
    std::shared_ptr<FieldLayout> fieldLayout;
};

class MultiTagCameraPoseEstimator : public CameraPoseEstimator {
public:
    explicit MultiTagCameraPoseEstimator(
        const CameraConfig& config = CameraConfig::Default(),
        std::shared_ptr<FieldLayout> layout = nullptr
    );
    
    CameraPoseObject SolveCameraPose(zarray_t* detections) override;

private:
    CameraPoseObject SolveSingleTag(apriltag_detection_t* det, const frc::Pose3d& tagPose);
    CameraPoseObject SolveMultiTag(
        const std::vector<cv::Point3d>& objectPoints,
        const std::vector<cv::Point2d>& framePoints,
        const std::vector<int>& tagIds
    );
};