#pragma once
#include <string>
#include <thread>
#include <atomic>
#include <memory>
#include <functional>
#include <vector>
#include "cameraFusion.hpp"
#include "multiCameraManager.hpp"

// Simple HTTP server for web interface
class WebServer {
public:
    WebServer(int port = 8080);
    ~WebServer();
    
    bool Start();
    void Stop();
    bool IsRunning() const;
    
    // Set callbacks to get current data
    void SetFusedPoseCallback(std::function<FusedPoseResult()> callback);
    void SetCameraResultsCallback(std::function<std::vector<CameraDetectionResult>()> callback);
    void SetStreamInfoCallback(std::function<std::vector<std::pair<std::string, int>>()> callback);

private:
    int port;
    std::atomic<bool> running;
    std::thread serverThread;
    
    std::function<FusedPoseResult()> fusedPoseCallback;
    std::function<std::vector<CameraDetectionResult>()> cameraResultsCallback;
    std::function<std::vector<std::pair<std::string, int>>()> streamInfoCallback;
    
    void ServerLoop();
    std::string HandleRequest(const std::string& request);
    
    // Route handlers
    std::string HandleIndex();
    std::string HandleAPIStatus();
    std::string HandleAPIFusedPose();
    std::string HandleAPICameras();
    std::string HandleAPIStreams();
    
    // HTML page generation
    std::string GenerateIndexPage();
};