#include "networkTablesPublisher.hpp"
#include "logger.hpp"

NetworkTablesPublisher::NetworkTablesPublisher(const NetworkTablesConfig& config)
    : config(config)
    , running(false) {
    
    ntInstance = nt::NetworkTableInstance::GetDefault();
}

NetworkTablesPublisher::~NetworkTablesPublisher() {
    Stop();
}

bool NetworkTablesPublisher::Start() {
    if (!config.enabled) {
        LOG_INFO("NetworkTables publishing disabled");
        return false;
    }
    
    if (running) return true;
    
    try {
        if (config.isServer) {
            // Server mode - run on coprocessor/laptop
            LOG_INFO("Starting NetworkTables server on port 5810");
            ntInstance.StartServer();
        } else {
            // Client mode - connect to robot
            if (!config.serverAddress.empty()) {
                LOG_INFO("Connecting NetworkTables client to " + config.serverAddress);
                ntInstance.StartClient4("vision-client");
                ntInstance.SetServer(config.serverAddress);
            } else if (!config.teamNumber.empty() && config.teamNumber != "0") {
                LOG_INFO("Connecting NetworkTables client to team " + config.teamNumber);
                ntInstance.StartClient4("vision-client");
                ntInstance.SetServerTeam(std::stoi(config.teamNumber));
            } else {
                LOG_ERROR("No server address or team number specified for client mode");
                return false;
            }
        }
        
        // Get tables
        visionTable = ntInstance.GetTable(config.tableName);
        fusedTable = visionTable->GetSubTable("Fused");
        camerasTable = visionTable->GetSubTable("Cameras");
        
        InitializePublishers();
        
        running = true;
        LOG_INFO("NetworkTables publisher started successfully");
        return true;
        
    } catch (const std::exception& e) {
        LOG_ERROR("Failed to start NetworkTables: " + std::string(e.what()));
        return false;
    }
}

void NetworkTablesPublisher::Stop() {
    if (!running) return;
    
    running = false;
    
    // Publish shutdown status
    if (systemActivePublisher) {
        systemActivePublisher.Set(false);
    }
    if (systemStatusPublisher) {
        systemStatusPublisher.Set("Shutdown");
    }
    
    // Flush all changes
    ntInstance.Flush();
    
    LOG_INFO("NetworkTables publisher stopped");
}

bool NetworkTablesPublisher::IsRunning() const {
    return running;
}

bool NetworkTablesPublisher::IsConnected() const {
    if (!running) return false;
    return ntInstance.IsConnected();
}

void NetworkTablesPublisher::InitializePublishers() {
    // Fused pose publishers
    fusedPosePublisher = fusedTable->GetDoubleArrayTopic("pose").Publish();
    fusedVelocityPublisher = fusedTable->GetDoubleArrayTopic("velocity").Publish();
    fusedConfidencePublisher = fusedTable->GetDoubleTopic("confidence").Publish();
    fusedCameraCountPublisher = fusedTable->GetIntegerTopic("camera_count").Publish();
    fusedTimestampPublisher = fusedTable->GetDoubleTopic("timestamp").Publish();
    
    // System status publishers
    systemActivePublisher = visionTable->GetBooleanTopic("active").Publish();
    systemStatusPublisher = visionTable->GetStringTopic("status").Publish();
    
    // Set initial status
    systemActivePublisher.Set(true);
    systemStatusPublisher.Set("Running");
    
    LOG_INFO("NetworkTables publishers initialized");
}

void NetworkTablesPublisher::InitializeCameraPublisher(const std::string& cameraName) {
    std::lock_guard<std::mutex> lock(cameraPublishersMutex);
    
    if (cameraPosePublishers.find(cameraName) != cameraPosePublishers.end()) {
        return; // Already initialized
    }
    
    auto cameraTable = camerasTable->GetSubTable(cameraName);
    
    // Pose estimation publishers
    cameraPosePublishers[cameraName] = 
        cameraTable->GetDoubleArrayTopic("pose").Publish();
    cameraTagIdPublishers[cameraName] = 
        cameraTable->GetIntegerArrayTopic("tag_ids").Publish();
    cameraErrorPublishers[cameraName] = 
        cameraTable->GetDoubleTopic("error").Publish();
    cameraActivePublishers[cameraName] = 
        cameraTable->GetBooleanTopic("active").Publish();
    
    // Tag angle publishers (NEW)
    auto angleTable = cameraTable->GetSubTable("Angles");
    cameraAngleTagIdPublishers[cameraName] = 
        angleTable->GetIntegerTopic("tag_id").Publish();
    cameraAngleCornersPublishers[cameraName] = 
        angleTable->GetDoubleArrayTopic("corners").Publish();
    cameraAngleDistancePublishers[cameraName] = 
        angleTable->GetDoubleTopic("distance").Publish();
    
    LOG_DEBUG("Initialized NetworkTables publishers for camera: " + cameraName);
}

void NetworkTablesPublisher::PublishFusedPose(const FusedPoseResult& result) {
    if (!running) return;
    
    try {
        // Publish pose [x, y, z, roll, pitch, yaw]
        auto poseArray = PoseToArray(result.pose);
        fusedPosePublisher.Set(poseArray);
        
        // Publish velocity [vx, vy, vz]
        std::vector<double> velocityArray = {
            result.velocity.x(),
            result.velocity.y(),
            result.velocity.z()
        };
        fusedVelocityPublisher.Set(velocityArray);
        
        // Publish confidence
        fusedConfidencePublisher.Set(result.confidence);
        
        // Publish camera count
        fusedCameraCountPublisher.Set(result.numCamerasUsed);
        
        // Publish timestamp (seconds since epoch)
        auto timestamp = std::chrono::duration<double>(
            result.timestamp.time_since_epoch()
        ).count();
        fusedTimestampPublisher.Set(timestamp);
        
        // Flush immediately for low latency
        ntInstance.Flush();
        
    } catch (const std::exception& e) {
        LOG_ERROR("Error publishing fused pose to NetworkTables: " + std::string(e.what()));
    }
}

void NetworkTablesPublisher::PublishCameraDetections(
    const std::vector<CameraDetectionResult>& results) {
    
    if (!running) return;
    
    std::lock_guard<std::mutex> lock(cameraPublishersMutex);
    
    try {
        for (const auto& result : results) {
            // Initialize publishers for new cameras
            if (cameraPosePublishers.find(result.cameraName) == cameraPosePublishers.end()) {
                InitializeCameraPublisher(result.cameraName);
            }
            
            // Mark camera as active
            cameraActivePublishers[result.cameraName].Set(true);
            
            // Publish pose data
            if (result.hasPose && result.poseData.isValid()) {
                // Publish pose
                auto poseArray = PoseToArray(result.poseData.pose_0);
                cameraPosePublishers[result.cameraName].Set(poseArray);
                
                // Publish tag IDs
                std::vector<int64_t> tagIds;
                for (int id : result.poseData.tag_ids) {
                    tagIds.push_back(id);
                }
                cameraTagIdPublishers[result.cameraName].Set(tagIds);
                
                // Publish error
                cameraErrorPublishers[result.cameraName].Set(result.poseData.error_0);
            } else {
                // No pose detections - publish empty arrays
                cameraTagIdPublishers[result.cameraName].Set(std::vector<int64_t>());
            }
            
            // Publish angle data (NEW)
            if (result.hasAngle && result.angleData.isValid()) {
                cameraAngleTagIdPublishers[result.cameraName].Set(result.angleData.tag_id);
                
                // Convert Eigen matrix to double array [corner0_az, corner0_el, corner1_az, corner1_el, ...]
                std::vector<double> cornersArray;
                for (int i = 0; i < 4; i++) {
                    cornersArray.push_back(result.angleData.corners(i, 0)); // azimuth
                    cornersArray.push_back(result.angleData.corners(i, 1)); // elevation
                }
                cameraAngleCornersPublishers[result.cameraName].Set(cornersArray);
                
                cameraAngleDistancePublishers[result.cameraName].Set(result.angleData.distance);
            }
        }
        
        ntInstance.Flush();
        
    } catch (const std::exception& e) {
        LOG_ERROR("Error publishing camera detections to NetworkTables: " + 
                  std::string(e.what()));
    }
}

void NetworkTablesPublisher::PublishAll(
    const FusedPoseResult& fusedResult,
    const std::vector<CameraDetectionResult>& cameraResults) {
    
    if (!running) return;
    
    PublishFusedPose(fusedResult);
    PublishCameraDetections(cameraResults);
}

std::vector<double> NetworkTablesPublisher::PoseToArray(const frc::Pose3d& pose) {
    return {
        pose.Translation().X().value(),
        pose.Translation().Y().value(),
        pose.Translation().Z().value(),
        pose.Rotation().X().value(), // roll
        pose.Rotation().Y().value(), // pitch
        pose.Rotation().Z().value()  // yaw
    };
}