#include "cameraCalibration.hpp"
#include <fstream>
#include <iostream>

std::optional<CameraConfig> CameraCalibration::LoadFromFile(const std::string& filepath) {
    cv::FileStorage fs(filepath, cv::FileStorage::READ);
    if (!fs.isOpened()) {
        std::cerr << "Failed to open calibration file: " << filepath << std::endl;
        return std::nullopt;
    }
    
    CameraConfig config;
    fs["camera_matrix"] >> config.cameraMatrix;
    fs["distortion_coefficients"] >> config.distCoeffs;
    
    if (fs["tag_size"].isReal()) {
        fs["tag_size"] >> config.tagSize;
    }
    
    fs.release();
    
    if (config.cameraMatrix.empty() || config.distCoeffs.empty()) {
        std::cerr << "Invalid calibration data in file: " << filepath << std::endl;
        return std::nullopt;
    }
    
    return config;
}

bool CameraCalibration::SaveToFile(const std::string& filepath, const CameraConfig& config) {
    cv::FileStorage fs(filepath, cv::FileStorage::WRITE);
    if (!fs.isOpened()) {
        std::cerr << "Failed to create calibration file: " << filepath << std::endl;
        return false;
    }
    
    fs << "camera_matrix" << config.cameraMatrix;
    fs << "distortion_coefficients" << config.distCoeffs;
    fs << "tag_size" << config.tagSize;
    
    fs.release();
    return true;
}

std::optional<CameraConfig> CameraCalibration::CalibrateFromChessboard(
    const std::vector<cv::Mat>& images,
    cv::Size boardSize,
    float squareSize) {
    
    if (images.empty()) {
        std::cerr << "No images provided for calibration" << std::endl;
        return std::nullopt;
    }
    
    std::vector<std::vector<cv::Point2f>> imagePoints;
    std::vector<std::vector<cv::Point3f>> objectPoints;
    
    // Generate 3D points
    std::vector<cv::Point3f> objp;
    for (int i = 0; i < boardSize.height; i++) {
        for (int j = 0; j < boardSize.width; j++) {
            objp.push_back(cv::Point3f(j * squareSize, i * squareSize, 0));
        }
    }
    
    // Find chessboard corners in each image
    for (const auto& img : images) {
        cv::Mat gray;
        if (img.channels() == 3) {
            cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);
        } else {
            gray = img.clone();
        }
        
        std::vector<cv::Point2f> corners;
        bool found = cv::findChessboardCorners(gray, boardSize, corners);
        
        if (found) {
            cv::cornerSubPix(gray, corners, cv::Size(11, 11), cv::Size(-1, -1),
                cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.1));
            imagePoints.push_back(corners);
            objectPoints.push_back(objp);
            std::cout << "Found chessboard in image " << objectPoints.size() << std::endl;
        }
    }
    
    if (imagePoints.empty()) {
        std::cerr << "No chessboard patterns found in any images" << std::endl;
        return std::nullopt;
    }
    
    std::cout << "Calibrating with " << imagePoints.size() << " images..." << std::endl;
    
    CameraConfig config;
    std::vector<cv::Mat> rvecs, tvecs;
    
    double rms = cv::calibrateCamera(objectPoints, imagePoints, images[0].size(),
                        config.cameraMatrix, config.distCoeffs, rvecs, tvecs);
    
    std::cout << "Calibration complete. RMS error: " << rms << std::endl;
    
    return config;
}