#pragma once
#include <string>
#include <memory>
#include <atomic>
#include <mutex>
#include <networktables/NetworkTableInstance.h>
#include <networktables/NetworkTable.h>
#include <networktables/DoubleArrayTopic.h>
#include <networktables/IntegerTopic.h>
#include <networktables/DoubleTopic.h>
#include <networktables/StringTopic.h>
#include <networktables/BooleanTopic.h>
#include <networktables/IntegerArrayTopic.h>
#include "cameraFusion.hpp"
#include "multiCameraManager.hpp"

struct NetworkTablesConfig {
    bool enabled = true;
    std::string teamNumber = "0";  // e.g., "254" for team 254
    std::string serverAddress = ""; // Empty for client mode, set for server mode
    bool isServer = false;
    std::string tableName = "Vision";
    double updateRate = 50.0; // Hz
};

class NetworkTablesPublisher {
public:
    explicit NetworkTablesPublisher(const NetworkTablesConfig& config);
    ~NetworkTablesPublisher();
    
    bool Start();
    void Stop();
    bool IsRunning() const;
    bool IsConnected() const;
    
    // Publish fused pose data
    void PublishFusedPose(const FusedPoseResult& result);
    
    // Publish individual camera detections
    void PublishCameraDetections(const std::vector<CameraDetectionResult>& results);
    
    // Publish both fused and individual data
    void PublishAll(
        const FusedPoseResult& fusedResult,
        const std::vector<CameraDetectionResult>& cameraResults
    );

private:
    NetworkTablesConfig config;
    std::atomic<bool> running;
    
    nt::NetworkTableInstance ntInstance;
    std::shared_ptr<nt::NetworkTable> visionTable;
    std::shared_ptr<nt::NetworkTable> fusedTable;
    std::shared_ptr<nt::NetworkTable> camerasTable;
    
    // Publishers for fused pose
    nt::DoubleArrayPublisher fusedPosePublisher;
    nt::DoubleArrayPublisher fusedVelocityPublisher;
    nt::DoublePublisher fusedConfidencePublisher;
    nt::IntegerPublisher fusedCameraCountPublisher;
    nt::DoublePublisher fusedTimestampPublisher;
    
    // Publishers for individual cameras
    std::mutex cameraPublishersMutex;
    std::unordered_map<std::string, nt::DoubleArrayPublisher> cameraPosePublishers;
    std::unordered_map<std::string, nt::IntegerArrayPublisher> cameraTagIdPublishers;
    std::unordered_map<std::string, nt::DoublePublisher> cameraErrorPublishers;
    std::unordered_map<std::string, nt::BooleanPublisher> cameraActivePublishers;
    
    // System status
    nt::BooleanPublisher systemActivePublisher;
    nt::StringPublisher systemStatusPublisher;
    
    void InitializePublishers();
    void InitializeCameraPublisher(const std::string& cameraName);
    
    // Convert pose to array format [x, y, z, roll, pitch, yaw]
    std::vector<double> PoseToArray(const frc::Pose3d& pose);
};