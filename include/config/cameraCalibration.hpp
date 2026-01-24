#pragma once
#include <opencv2/opencv.hpp>
#include <string>
#include <optional>
#include <vector>
#include "dataTypes.hpp"

class CameraCalibration {
public:
    // Load calibration from OpenCV XML/YAML format
    static std::optional<CameraConfig> LoadFromFile(const std::string& filepath);
    
    // Save calibration to file
    static bool SaveToFile(const std::string& filepath, const CameraConfig& config);
    
    // Perform camera calibration using chessboard images
    static std::optional<CameraConfig> CalibrateFromChessboard(
        const std::vector<cv::Mat>& images,
        cv::Size boardSize,
        float squareSize
    );
};