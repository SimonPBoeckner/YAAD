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
        
        // Add cameras (example with 2 cameras)
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
        
        cameraManager.AddCamera(cam1Config);
        
        // Optionally add second camera
        // CameraStreamConfig cam2Config = cam1Config;
        // cam2Config.cameraIndex = 1;
        // cam2Config.cameraName = "rear_camera";
        // cameraManager.AddCamera(cam2Config);
        
        // Initialize camera fusion
        CameraFusion fusion;
        fusion.SetMaxMeasurementAge(0.5);
        fusion.SetMinCamerasRequired(1);
        fusion.SetProcessNoise(0.01);
        
        // Initialize network streamer
        NetworkConfig networkConfig;
        networkConfig.protocol = NetworkProtocol::UDP;
        networkConfig.address = "127.0.0.1";
        networkConfig.port = 5800;
        
        NetworkStreamer networkStreamer(networkConfig);
        if (networkStreamer.Start()) {
            LOG_INFO("Network streamer started on UDP port 5800");
        }
        
        // Initialize NetworkTables publisher for FRC
        NetworkTablesConfig ntConfig;
        ntConfig.enabled = config.networkTablesEnabled;
        ntConfig.teamNumber = config.teamNumber;
        ntConfig.serverAddress = config.ntServerAddress;
        ntConfig.isServer = config.ntIsServer;
        ntConfig.tableName = config.ntTableName;
        
        NetworkTablesPublisher ntPublisher(ntConfig);
        if (ntPublisher.Start()) {
            if (config.ntIsServer) {
                LOG_INFO("NetworkTables server started on port 5810");
            } else if (!config.teamNumber.empty() && config.teamNumber != "0") {
                LOG_INFO("NetworkTables client connecting to team " + config.teamNumber);
            } else if (!config.ntServerAddress.empty()) {
                LOG_INFO("NetworkTables client connecting to " + config.ntServerAddress);
            }
        }
        
        // Initialize MJPEG stream manager
        MJPEGStreamManager mjpegManager;
        mjpegManager.AddStream("front_camera", 8081);
        // Add more streams as needed for additional cameras
        mjpegManager.StartAll();
        LOG_INFO("MJPEG streams started");
        
        // Initialize web server
        WebServer webServer(8080);
        
        // Latest fused result storage
        FusedPoseResult latestFusedResult;
        std::mutex fusedResultMutex;
        
        // Set web server callbacks
        webServer.SetFusedPoseCallback([&]() -> FusedPoseResult {
            std::lock_guard<std::mutex> lock(fusedResultMutex);
            return latestFusedResult;
        });
        
        webServer.SetCameraResultsCallback([&]() -> std::vector<CameraDetectionResult> {
            return cameraManager.GetAllLatestResults();
        });
        
        webServer.SetStreamInfoCallback([&]() -> std::vector<std::pair<std::string, int>> {
            return mjpegManager.GetStreamInfo();
        });
        
        if (webServer.Start()) {
            LOG_INFO("Web interface available at http://localhost:8080");
        }
        
        // Start all cameras
        cameraManager.StartAll();
        LOG_INFO("All cameras started");
        
        // Performance tracking
        FPSCounter fpsCounter;
        PerformanceMonitor& perfMon = PerformanceMonitor::Instance();
        int frameCount = 0;
        
        LOG_INFO("System running. Press Ctrl+C to exit.");
        LOG_INFO("Web interface: http://localhost:8080");
        if (ntPublisher.IsRunning()) {
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
            
            // Update MJPEG streams with frames
            for (const auto& result : cameraResults) {
                if (result.hasFrame && !result.frame.empty()) {
                    mjpegManager.UpdateFrame(result.cameraName, result.frame);
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
            
            // Send over network
            if (networkStreamer.IsRunning()) {
                networkStreamer.SendFusedPose(fusedResult);
                networkStreamer.SendMultipleCameras(cameraResults);
            }
            
            // Publish to NetworkTables for FRC
            if (ntPublisher.IsRunning()) {
                ntPublisher.PublishAll(fusedResult, cameraResults);
            }
            
            // Log results periodically
            if (frameCount % 30 == 0 && fusedResult.confidence > 0.5) {
                std::string ntStatus = ntPublisher.IsConnected() ? "Connected" : "Disconnected";
                LOG_INFO("Fused pose: [" +
                    std::to_string(fusedResult.pose.Translation().X().value()) + ", " +
                    std::to_string(fusedResult.pose.Translation().Y().value()) + ", " +
                    std::to_string(fusedResult.pose.Translation().Z().value()) + "] " +
                    "Confidence: " + std::to_string(fusedResult.confidence) +
                    " Cameras: " + std::to_string(fusedResult.numCamerasUsed) +
                    " NT: " + ntStatus
                );
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
            
            // Don't spin too fast
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        
        // Cleanup
        LOG_INFO("Shutting down...");
        ntPublisher.Stop();
        cameraManager.StopAll();
        mjpegManager.StopAll();
        networkStreamer.Stop();
        webServer.Stop();
        
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