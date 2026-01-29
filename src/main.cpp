#include <iostream>
#include <memory>
#include <csignal>
#include <atomic>

#include "multiCameraManager.hpp"
#include "cameraFusion.hpp"
#include "networkStreamer.hpp"
#include "networkTablesPublisher.hpp"
#include "webServer.hpp"
#include "mjpegStreamer.hpp"
#include "configManager.hpp"
#include "logger.hpp"
#include "performanceMonitor.hpp"

std::atomic<bool> running(true);

void signalHandler(int signum) {
    LOG_INFO("Interrupt signal received. Shutting down...");
    running = false;
}

int main(int argc, char** argv) {
    signal(SIGINT, signalHandler);
    
    try {
        // Load configuration
        std::string configPath = (argc > 1) ? argv[1] : "config.json";
        auto configOpt = ConfigManager::LoadFromFile(configPath);
        AppConfig config = configOpt.value_or(ConfigManager::CreateDefault());
        
        // Setup logger
        Logger::Instance().SetLogFile(config.logFilePath);
        Logger::Instance().SetLevel(config.verboseLogging ? LogLevel::DEBUG : LogLevel::INFO);
        
        LOG_INFO("=== AprilTag Multi-Camera Vision System ===");
        LOG_INFO("Starting advanced vision system with fusion and networking...");
        
        // Load field layout
        std::shared_ptr<FieldLayout> fieldLayout;
        if (!config.fieldLayoutPath.empty()) {
            try {
                fieldLayout = std::make_shared<FieldLayout>(config.fieldLayoutPath);
                LOG_INFO("Field layout loaded: " + config.fieldLayoutPath);
            } catch (const std::exception& e) {
                LOG_ERROR("Failed to load field layout: " + std::string(e.what()));
                return 1;
            }
        }
        
        // Initialize multi-camera manager
        MultiCameraManager cameraManager(fieldLayout);
        
        // Add cameras (example with 1 camera, configure based on your needs)
        CameraStreamConfig cam1Config;
        cam1Config.cameraIndex = 0;
        cam1Config.cameraName = "front_camera";
        cam1Config.captureConfig.width = config.cameraWidth;
        cam1Config.captureConfig.height = config.cameraHeight;
        cam1Config.captureConfig.fps = config.cameraFPS;
        cam1Config.detectorConfig.quad_decimate = config.quadDecimate;
        cam1Config.detectorConfig.nthreads = config.numThreads;
        cam1Config.cameraConfig = CameraConfig::Default();
        cam1Config.cameraConfig.tagSize = config.tagSize;
        cam1Config.cameraMountPose = frc::Pose3d(); // Identity pose
        
        // Enable features based on config
        cam1Config.enablePoseEstimation = true;
        cam1Config.enableAngleCalculation = false;  // Toggle this per camera
        cam1Config.storeFrames = config.showVisualization;  // Only store frames if visualization is enabled
        
        cameraManager.AddCamera(cam1Config);
        
        // Initialize camera fusion
        CameraFusion fusion;
        fusion.SetMaxMeasurementAge(0.5);
        fusion.SetMinCamerasRequired(1);
        fusion.SetProcessNoise(0.01);
        
        // Latest fused result storage (needed for web server if enabled)
        FusedPoseResult latestFusedResult;
        std::mutex fusedResultMutex;
        
        // Initialize network streamer (if enabled in config)
        std::unique_ptr<NetworkStreamer> networkStreamer;
        bool networkEnabled = false;
        
        // Check if network section exists in config
        // For now, we'll add a flag to AppConfig to control this
        // Defaulting to disabled to match your intent
        if (networkEnabled) {  // TODO: Add networkEnabled flag to AppConfig
            NetworkConfig networkConfig;
            networkConfig.protocol = NetworkProtocol::UDP;
            networkConfig.address = "127.0.0.1";
            networkConfig.port = 5800;
            
            networkStreamer = std::make_unique<NetworkStreamer>(networkConfig);
            if (networkStreamer->Start()) {
                LOG_INFO("Network streamer started on UDP port 5800");
            }
        } else {
            LOG_INFO("Network streaming disabled in config");
        }
        
        // Initialize NetworkTables publisher for FRC
        NetworkTablesConfig ntConfig;
        ntConfig.enabled = config.networkTablesEnabled;
        ntConfig.teamNumber = config.teamNumber;
        ntConfig.serverAddress = config.ntServerAddress;
        ntConfig.isServer = config.ntIsServer;
        ntConfig.tableName = config.ntTableName;
        
        NetworkTablesPublisher ntPublisher(ntConfig);
        bool ntStarted = false;
        
        if (config.networkTablesEnabled) {
            LOG_INFO("Attempting to start NetworkTables...");
            ntStarted = ntPublisher.Start();
            
            if (ntStarted) {
                if (config.ntIsServer) {
                    LOG_INFO("NetworkTables server started on port 5810");
                } else if (!config.teamNumber.empty() && config.teamNumber != "0") {
                    LOG_INFO("NetworkTables client connecting to team " + config.teamNumber);
                } else if (!config.ntServerAddress.empty()) {
                    LOG_INFO("NetworkTables client connecting to " + config.ntServerAddress);
                }
            } else {
                LOG_WARNING("NetworkTables failed to start - continuing without it");
            }
        } else {
            LOG_INFO("NetworkTables disabled in config");
        }
        
        // Initialize MJPEG stream manager (if visualization enabled)
        std::unique_ptr<MJPEGStreamManager> mjpegManager;
        if (config.showVisualization && cam1Config.storeFrames) {
            mjpegManager = std::make_unique<MJPEGStreamManager>();
            mjpegManager->AddStream("front_camera", 8081);
            // Add more streams as needed for additional cameras
            mjpegManager->StartAll();
            LOG_INFO("MJPEG streams started");
        } else {
            LOG_INFO("MJPEG streaming disabled (visualization off or storeFrames = false)");
        }
        
        // Initialize web server (if enabled in config)
        std::unique_ptr<WebServer> webServer;
        if (config.showVisualization) {  // Use showVisualization to control web server
            webServer = std::make_unique<WebServer>(8080);
            
            // Set web server callbacks
            webServer->SetFusedPoseCallback([&]() -> FusedPoseResult {
                std::lock_guard<std::mutex> lock(fusedResultMutex);
                return latestFusedResult;
            });
            
            webServer->SetCameraResultsCallback([&]() -> std::vector<CameraDetectionResult> {
                return cameraManager.GetAllLatestResults();
            });
            
            webServer->SetStreamInfoCallback([&]() -> std::vector<std::pair<std::string, int>> {
                if (mjpegManager) {
                    return mjpegManager->GetStreamInfo();
                }
                return std::vector<std::pair<std::string, int>>();
            });
            
            if (webServer->Start()) {
                LOG_INFO("Web interface available at http://localhost:8080");
            }
        } else {
            LOG_INFO("Web server disabled in config");
        }
        
        // Start all cameras
        cameraManager.StartAll();
        LOG_INFO("All cameras started");
        
        // Performance tracking
        FPSCounter fpsCounter;
        PerformanceMonitor& perfMon = PerformanceMonitor::Instance();
        int frameCount = 0;
        
        LOG_INFO("System running. Press Ctrl+C to exit.");
        if (config.showVisualization && webServer) {
            LOG_INFO("Web interface: http://localhost:8080");
        }
        if (ntStarted && ntPublisher.IsRunning()) {
            LOG_INFO("NetworkTables publishing to table: " + config.ntTableName);
        }
        
        // Main processing loop
        while (running) {
            if (config.enablePerformanceMonitoring) {
                PERF_TIMER("main_loop");
            }
            
            // Get detections from all cameras
            std::vector<CameraDetectionResult> cameraResults;
            {
                if (config.enablePerformanceMonitoring) {
                    PERF_TIMER("get_camera_results");
                }
                cameraResults = cameraManager.GetAllLatestResults();
            }
            
            // Update MJPEG streams with frames ONLY if visualization enabled and frames available
            if (config.showVisualization && mjpegManager) {
                for (const auto& result : cameraResults) {
                    if (result.hasFrame && !result.frame.empty()) {
                        mjpegManager->UpdateFrame(result.cameraName, result.frame);
                    }
                }
            }
            
            // Fuse detections
            FusedPoseResult fusedResult;
            {
                if (config.enablePerformanceMonitoring) {
                    PERF_TIMER("fusion");
                }
                fusedResult = fusion.FuseDetections(cameraResults);
            }
            
            // Update latest result for web interface
            {
                std::lock_guard<std::mutex> lock(fusedResultMutex);
                latestFusedResult = fusedResult;
            }
            
            // Send over network (if enabled)
            if (networkStreamer && networkStreamer->IsRunning()) {
                networkStreamer->SendFusedPose(fusedResult);
                networkStreamer->SendMultipleCameras(cameraResults);
            }
            
            // Publish to NetworkTables for FRC
            if (ntPublisher.IsRunning()) {
                if (config.enablePerformanceMonitoring) {
                    PERF_TIMER("nt_publish");
                }
                
                ntPublisher.PublishAll(fusedResult, cameraResults);
                
                // Flush every frame for real-time updates
                ntPublisher.Flush();
                
                // More frequent status logging for debugging
                if (frameCount % 100 == 0) {
                    bool connected = ntPublisher.IsConnected();
                    LOG_INFO("NT Status: " + std::string(connected ? "CONNECTED" : "DISCONNECTED") + 
                            " | Frame: " + std::to_string(frameCount) +
                            " | Cameras: " + std::to_string(cameraResults.size()));
                }
            }
            
            // Log results periodically with detailed debugging
            if (frameCount % 30 == 0) {
                std::string ntStatus = ntPublisher.IsConnected() ? "Connected" : "Disconnected";
                
                // Debug: Log camera results
                LOG_INFO("=== Frame " + std::to_string(frameCount) + " Debug ===");
                LOG_INFO("Camera results count: " + std::to_string(cameraResults.size()));
                
                for (const auto& result : cameraResults) {
                    LOG_INFO("Camera: " + result.cameraName + 
                            " | hasPose: " + std::to_string(result.hasPose) +
                            " | hasAngle: " + std::to_string(result.hasAngle));
                    
                    if (result.hasPose && result.poseData.isValid()) {
                        LOG_INFO("  Tags detected: " + std::to_string(result.poseData.tag_ids.size()));
                        std::string tagIds = "  Tag IDs: ";
                        for (int id : result.poseData.tag_ids) {
                            tagIds += std::to_string(id) + " ";
                        }
                        LOG_INFO(tagIds);
                        LOG_INFO("  Pose: [" +
                            std::to_string(result.poseData.pose_0.Translation().X().value()) + ", " +
                            std::to_string(result.poseData.pose_0.Translation().Y().value()) + ", " +
                            std::to_string(result.poseData.pose_0.Translation().Z().value()) + "]");
                        LOG_INFO("  Error: " + std::to_string(result.poseData.error_0));
                    } else {
                        LOG_INFO("  No valid pose data");
                    }
                }
                
                if (fusedResult.confidence > 0.5) {
                    LOG_INFO("Fused pose: [" +
                        std::to_string(fusedResult.pose.Translation().X().value()) + ", " +
                        std::to_string(fusedResult.pose.Translation().Y().value()) + ", " +
                        std::to_string(fusedResult.pose.Translation().Z().value()) + "] " +
                        "Confidence: " + std::to_string(fusedResult.confidence) +
                        " Cameras: " + std::to_string(fusedResult.numCamerasUsed) +
                        " NT: " + ntStatus
                    );
                } else {
                    LOG_INFO("Low confidence (" + std::to_string(fusedResult.confidence) + ") | NT: " + ntStatus);
                }
                
                LOG_INFO("================================");
            } else if (frameCount % 100 == 0) {
                // Less frequent status update
                std::string ntStatus = ntPublisher.IsConnected() ? "Connected" : "Disconnected";
                LOG_INFO("System running | NT: " + ntStatus + " | Frame: " + std::to_string(frameCount));
            }
            
            // Update FPS
            fpsCounter.Tick();
            frameCount++;
            
            // Performance reporting
            if (config.enablePerformanceMonitoring && 
                frameCount % config.performanceReportInterval == 0) {
                perfMon.PrintStats();
                LOG_INFO("FPS: " + std::to_string(fpsCounter.GetFPS()));
            }
            
            // Don't spin too fast - adjust based on your needs
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        
        // Cleanup
        LOG_INFO("Shutting down...");
        ntPublisher.Stop();
        cameraManager.StopAll();
        if (mjpegManager) {
            mjpegManager->StopAll();
        }
        if (networkStreamer) {
            networkStreamer->Stop();
        }
        if (webServer) {
            webServer->Stop();
        }
        
        if (config.enablePerformanceMonitoring) {
            LOG_INFO("Final performance statistics:");
            perfMon.PrintStats();
        }
        
        LOG_INFO("System shutdown complete. Processed " + std::to_string(frameCount) + " frames.");
        return 0;

    } catch (const std::exception& e) {
        LOG_CRITICAL("Fatal error: " + std::string(e.what()));
        return 1;
    }
}