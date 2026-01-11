#pragma once
#include <apriltag/apriltag.h>
#include <opencv2/opencv.hpp>
#include "frc/geometry/Pose3d.h"
#include <vector>
#include <nlohmann/json.hpp>

class CameraPoseEstimator {
public:
    CameraPoseEstimator();
    virtual ~CameraPoseEstimator();

    virtual void SolveCameraPose(zarray_t* detections);

protected:
    nlohmann::json jason;

    float fid_size;
    std::vector<std::vector<units::meter_t>> object_points;
    std::vector<std::vector<units::meter_t>> frame_points;
    std::vector<int> tag_ids;
    std::vector<frc::Pose3d> tag_poses;
};

class MultiTagCameraPoseEstimator: public CameraPoseEstimator {
public:
    void SolveCameraPose(zarray_t* detections) override;
};