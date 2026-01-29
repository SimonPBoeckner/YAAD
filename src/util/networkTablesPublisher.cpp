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
        LOG_INFO("Starting NetworkTables initialization...");
        
        if (config.isServer) {
            // Server mode - run on coprocessor/laptop
            LOG_INFO("Starting NetworkTables server on port 5810");
            ntInstance.StartServer();
        } else {
            // Client mode - connect to robot
            LOG_INFO("Starting NetworkTables client...");
            ntInstance.StartClient4("vision-client");
            
            if (!config.serverAddress.empty()) {
                LOG_INFO("Setting server address: " + config.serverAddress);
                ntInstance.SetServer(config.serverAddress.c_str(), 5810);
            } else if (!config.teamNumber.empty() && config.teamNumber != "0") {
                LOG_INFO("Setting team number: " + config.teamNumber);
                ntInstance.SetServerTeam(std::stoi(config.teamNumber), 5810);
            } else {
                LOG_ERROR("No server address or team number specified for client mode");
                return false;
            }
        }
        
        // Small delay to allow connection setup
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        
        // Get tables
        visionTable = ntInstance.GetTable(config.tableName);
        fusedTable = visionTable->GetSubTable("Fused");
        camerasTable = visionTable->GetSubTable("Cameras");
        
        // Initialize publishers with defaults
        InitializePublishers();
        
        running = true;
        
        // Wait for connection (up to 2 seconds)
        LOG_INFO("Waiting for NetworkTables connection...");
        for (int i = 0; i < 20; i++) {
            if (IsConnected()) {
                LOG_INFO("NetworkTables connected successfully!");
                return true;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        
        if (!IsConnected()) {
            LOG_WARNING("NetworkTables started but not yet connected - will continue trying in background");
        }
        
        return true;
        
    } catch (const std::exception& e) {
        LOG_ERROR("Failed to start NetworkTables: " + std::string(e.what()));
        running = false;
        return false;
    }
}

void NetworkTablesPublisher::Stop() {
    if (!running) return;
    
    running = false;
    
    // Publish shutdown status
    try {
        if (systemActivePublisher) {
            systemActivePublisher.Set(false);
        }
        if (systemStatusPublisher) {
            systemStatusPublisher.Set("Shutdown");
        }
        
        // Flush all changes
        ntInstance.Flush();
    } catch (const std::exception& e) {
        LOG_ERROR("Error during NetworkTables shutdown: " + std::string(e.what()));
    }
    
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
    try {
        // Fused pose publishers with default values
        nt::PubSubOptions poseOptions;
        poseOptions.periodic = 0.02; // 50Hz update rate
        poseOptions.sendAll = true;
        
        fusedPosePublisher = fusedTable->GetDoubleArrayTopic("pose").Publish(poseOptions);
        fusedPosePublisher.SetDefault(std::vector<double>{0, 0, 0, 0, 0, 0});
        fusedPosePublisher.Set(std::vector<double>{0, 0, 0, 0, 0, 0});
        
        fusedVelocityPublisher = fusedTable->GetDoubleArrayTopic("velocity").Publish(poseOptions);
        fusedVelocityPublisher.SetDefault(std::vector<double>{0, 0, 0});
        fusedVelocityPublisher.Set(std::vector<double>{0, 0, 0});
        
        fusedConfidencePublisher = fusedTable->GetDoubleTopic("confidence").Publish(poseOptions);
        fusedConfidencePublisher.SetDefault(0.0);
        fusedConfidencePublisher.Set(0.0);
        
        fusedCameraCountPublisher = fusedTable->GetIntegerTopic("camera_count").Publish(poseOptions);
        fusedCameraCountPublisher.SetDefault(0);
        fusedCameraCountPublisher.Set(0);
        
        fusedTimestampPublisher = fusedTable->GetDoubleTopic("timestamp").Publish(poseOptions);
        fusedTimestampPublisher.SetDefault(0.0);
        fusedTimestampPublisher.Set(0.0);
        
        // System status publishers
        systemActivePublisher = visionTable->GetBooleanTopic("active").Publish(poseOptions);
        systemActivePublisher.SetDefault(false);
        systemActivePublisher.Set(true);
        
        systemStatusPublisher = visionTable->GetStringTopic("status").Publish(poseOptions);
        systemStatusPublisher.SetDefault("Initializing");
        systemStatusPublisher.Set("Running");
        
        // Force immediate flush to publish defaults
        ntInstance.Flush();
        
        LOG_INFO("NetworkTables publishers initialized with default values");
    } catch (const std::exception& e) {
        LOG_ERROR("Failed to initialize NetworkTables publishers: " + std::string(e.what()));
        throw;
    }
}

void NetworkTablesPublisher::InitializeCameraPublisher(const std::string& cameraName) {
    std::lock_guard<std::mutex> lock(cameraPublishersMutex);
    
    if (cameraPosePublishers.find(cameraName) != cameraPosePublishers.end()) {
        return; // Already initialized
    }
    
    try {
        auto cameraTable = camerasTable->GetSubTable(cameraName);
        
        nt::PubSubOptions options;
        options.periodic = 0.02; // 50Hz
        options.sendAll = true;
        
        // Pose estimation publishers with defaults
        cameraPosePublishers[cameraName] = 
            cameraTable->GetDoubleArrayTopic("pose").Publish(options);
        cameraPosePublishers[cameraName].SetDefault(std::vector<double>{0, 0, 0, 0, 0, 0});
        cameraPosePublishers[cameraName].Set(std::vector<double>{0, 0, 0, 0, 0, 0});
        
        cameraTagIdPublishers[cameraName] = 
            cameraTable->GetIntegerArrayTopic("tag_ids").Publish(options);
        cameraTagIdPublishers[cameraName].SetDefault(std::vector<int64_t>{});
        cameraTagIdPublishers[cameraName].Set(std::vector<int64_t>{});
        
        cameraErrorPublishers[cameraName] = 
            cameraTable->GetDoubleTopic("error").Publish(options);
        cameraErrorPublishers[cameraName].SetDefault(0.0);
        cameraErrorPublishers[cameraName].Set(0.0);
        
        cameraActivePublishers[cameraName] = 
            cameraTable->GetBooleanTopic("active").Publish(options);
        cameraActivePublishers[cameraName].SetDefault(false);
        cameraActivePublishers[cameraName].Set(true);
        
        // Tag angle publishers with defaults
        auto angleTable = cameraTable->GetSubTable("Angles");
        cameraAngleTagIdPublishers[cameraName] = 
            angleTable->GetIntegerTopic("tag_id").Publish(options);
        cameraAngleTagIdPublishers[cameraName].SetDefault(-1);
        cameraAngleTagIdPublishers[cameraName].Set(-1);
        
        cameraAngleCornersPublishers[cameraName] = 
            angleTable->GetDoubleArrayTopic("corners").Publish(options);
        cameraAngleCornersPublishers[cameraName].SetDefault(std::vector<double>(8, 0.0));
        cameraAngleCornersPublishers[cameraName].Set(std::vector<double>(8, 0.0));
        
        cameraAngleDistancePublishers[cameraName] = 
            angleTable->GetDoubleTopic("distance").Publish(options);
        cameraAngleDistancePublishers[cameraName].SetDefault(0.0);
        cameraAngleDistancePublishers[cameraName].Set(0.0);
        
        // Flush immediately
        ntInstance.Flush();
        
        LOG_DEBUG("Initialized NetworkTables publishers for camera: " + cameraName);
    } catch (const std::exception& e) {
        LOG_ERROR("Failed to initialize publishers for camera " + cameraName + ": " + std::string(e.what()));
    }
}

void NetworkTablesPublisher::PublishFusedPose(const FusedPoseResult& result) {
    if (!running) return;
    
    try {
        // Validate publisher exists
        if (!fusedPosePublisher) {
            LOG_ERROR("Fused pose publisher not initialized");
            return;
        }
        
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
        
        // Debug logging every 30 frames
        static int publishCount = 0;
        if (++publishCount % 30 == 0) {
            LOG_INFO("NT Fused: pos=[" + 
                std::to_string(poseArray[0]) + ", " +
                std::to_string(poseArray[1]) + ", " +
                std::to_string(poseArray[2]) + "] conf=" +
                std::to_string(result.confidence) + " cams=" +
                std::to_string(result.numCamerasUsed) + " connected=" +
                (IsConnected() ? "YES" : "NO"));
        }
        
    } catch (const std::exception& e) {
        LOG_ERROR("Error publishing fused pose to NetworkTables: " + std::string(e.what()));
    }
}

void NetworkTablesPublisher::PublishCameraDetections(
    const std::vector<CameraDetectionResult>& results) {
    
    if (!running) return;
    
    std::lock_guard<std::mutex> lock(cameraPublishersMutex);
    
    try {
        static int publishCount = 0;
        publishCount++;
        
        for (const auto& result : results) {
            // Initialize publishers for new cameras
            if (cameraPosePublishers.find(result.cameraName) == cameraPosePublishers.end()) {
                LOG_INFO("Initializing publishers for new camera: " + result.cameraName);
                InitializeCameraPublisher(result.cameraName);
            }
            
            // Validate publishers exist
            auto poseIt = cameraPosePublishers.find(result.cameraName);
            if (poseIt == cameraPosePublishers.end()) {
                LOG_WARNING("Publishers not initialized for camera: " + result.cameraName);
                continue;
            }
            
            // Mark camera as active
            cameraActivePublishers[result.cameraName].Set(true);
            
            // Publish pose data
            if (result.hasPose && result.poseData.isValid()) {
                // Debug log
                if (publishCount % 30 == 0) {
                    LOG_INFO("NT Camera " + result.cameraName + 
                        " tags=[" + 
                        [&]() {
                            std::string ids;
                            for (size_t i = 0; i < result.poseData.tag_ids.size(); i++) {
                                if (i > 0) ids += ",";
                                ids += std::to_string(result.poseData.tag_ids[i]);
                            }
                            return ids;
                        }() +
                        "] error=" + std::to_string(result.poseData.error_0));
                }
                
                // Publish pose
                auto poseArray = PoseToArray(result.poseData.pose_0);
                poseIt->second.Set(poseArray);
                
                // Publish tag IDs
                std::vector<int64_t> tagIds;
                tagIds.reserve(result.poseData.tag_ids.size());
                for (int id : result.poseData.tag_ids) {
                    tagIds.push_back(static_cast<int64_t>(id));
                }
                cameraTagIdPublishers[result.cameraName].Set(tagIds);
                
                // Publish error
                cameraErrorPublishers[result.cameraName].Set(result.poseData.error_0);
            } else {
                // No pose detections - publish empty arrays
                if (publishCount % 60 == 0) {
                    LOG_DEBUG("NT Camera " + result.cameraName + " no valid pose");
                }
                cameraTagIdPublishers[result.cameraName].Set(std::vector<int64_t>());
                cameraErrorPublishers[result.cameraName].Set(0.0);
            }
            
            // Publish angle data
            if (result.hasAngle && result.angleData.isValid()) {
                cameraAngleTagIdPublishers[result.cameraName].Set(result.angleData.tag_id);
                
                // Convert Eigen matrix to double array [corner0_az, corner0_el, ...]
                std::vector<double> cornersArray;
                cornersArray.reserve(8);
                for (int i = 0; i < 4; i++) {
                    cornersArray.push_back(result.angleData.corners(i, 0)); // azimuth
                    cornersArray.push_back(result.angleData.corners(i, 1)); // elevation
                }
                cameraAngleCornersPublishers[result.cameraName].Set(cornersArray);
                
                cameraAngleDistancePublishers[result.cameraName].Set(result.angleData.distance);
            } else {
                // No angle data - set defaults
                cameraAngleTagIdPublishers[result.cameraName].Set(-1);
                cameraAngleDistancePublishers[result.cameraName].Set(0.0);
            }
        }
        
        // Log connection status periodically
        if (publishCount % 30 == 0) {
            LOG_INFO("NT: Published frame " + std::to_string(publishCount) + 
                    " | Connected: " + (IsConnected() ? "YES" : "NO") +
                    " | Cameras: " + std::to_string(results.size()));
        }
        
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

void NetworkTablesPublisher::Flush() {
    if (!running) return;
    
    try {
        ntInstance.Flush();
    } catch (const std::exception& e) {
        // Ignore flush errors - not critical
    }
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