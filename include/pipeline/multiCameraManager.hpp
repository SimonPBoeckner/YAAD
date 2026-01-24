#pragma once
#include <vector>
#include <thread>
#include <mutex>
#include <atomic>
#include <memory>
#include <queue>
#include <condition_variable>
#include "capture.hpp"
#include "fiducialDetector.hpp"
#include "cameraPoseEstimator.hpp"
#include "tagAngleCalculator.hpp"
#include "performanceMonitor.hpp"
#include "dataTypes.hpp"

struct CameraStreamConfig {
    int cameraIndex;
    std::string cameraName;
    CaptureConfig captureConfig;
    DetectorConfig detectorConfig;
    CameraConfig cameraConfig;
    frc::Pose3d cameraMountPose; // Position of camera relative to robot/world
    
    // Toggle features
    bool enablePoseEstimation = true;
    bool enableAngleCalculation = false;  // NEW: Enable tag angle output
};

struct CameraDetectionResult {
    std::string cameraName;
    int cameraIndex;
    CameraPoseObject poseData;
    TagAngleObject angleData;  // NEW: Tag angle data
    std::chrono::system_clock::time_point timestamp;
    cv::Mat frame; // Optional, for visualization
    bool hasFrame;
    bool hasPose;
    bool hasAngle;  // NEW: Flag for angle data validity
};

class CameraStream {
public:
    CameraStream(
        const CameraStreamConfig& config,
        std::shared_ptr<FieldLayout> layout
    );
    ~CameraStream();
    
    void Start();
    void Stop();
    bool IsRunning() const;
    
    // Thread-safe access to latest result
    std::optional<CameraDetectionResult> GetLatestResult();
    
    const std::string& GetName() const { return config.cameraName; }

private:
    void ProcessingLoop();
    void DrawAngleVisualization(cv::Mat& frame, const TagAngleObject& angleData);
    
    CameraStreamConfig config;
    std::shared_ptr<FieldLayout> fieldLayout;
    
    std::unique_ptr<DefaultCapture> capture;
    std::unique_ptr<FiducialDetector> detector;
    std::unique_ptr<MultiTagCameraPoseEstimator> poseEstimator;
    std::unique_ptr<CameraMatrixTagAngleCalculator> angleCalculator;  // NEW
    
    std::thread workerThread;
    std::atomic<bool> running;
    
    // Thread-safe result storage
    mutable std::mutex resultMutex;
    CameraDetectionResult latestResult;
    bool hasResult;
};

class MultiCameraManager {
public:
    MultiCameraManager(std::shared_ptr<FieldLayout> layout);
    ~MultiCameraManager();
    
    void AddCamera(const CameraStreamConfig& config);
    void StartAll();
    void StopAll();
    
    // Get results from all cameras
    std::vector<CameraDetectionResult> GetAllLatestResults();
    
    // Get result from specific camera
    std::optional<CameraDetectionResult> GetCameraResult(const std::string& cameraName);
    
    size_t GetCameraCount() const;

private:
    std::shared_ptr<FieldLayout> fieldLayout;
    std::vector<std::unique_ptr<CameraStream>> cameras;
    mutable std::mutex camerasMutex;
};