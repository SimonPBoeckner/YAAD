#pragma once
#include <string>
#include <memory>
#include <thread>
#include <atomic>
#include <mutex>
#include <queue>
#include "cameraFusion.hpp"
#include <nlohmann/json.hpp>

// Platform-independent socket handling
#ifdef _WIN32
    #include <winsock2.h>
    #include <ws2tcpip.h>
    typedef SOCKET SocketType;
    #define INVALID_SOCKET_VALUE INVALID_SOCKET
#else
    #include <sys/socket.h>
    #include <netinet/in.h>
    #include <arpa/inet.h>
    #include <unistd.h>
    typedef int SocketType;
    #define INVALID_SOCKET_VALUE -1
#endif

enum class NetworkProtocol {
    UDP,
    TCP
};

struct NetworkConfig {
    NetworkProtocol protocol = NetworkProtocol::UDP;
    std::string address = "127.0.0.1";
    int port = 5800;
    int maxQueueSize = 100;
};

// Serialize detection results to JSON
class DetectionSerializer {
public:
    static nlohmann::json SerializeFusedPose(const FusedPoseResult& result);
    static nlohmann::json SerializeCameraDetection(const CameraDetectionResult& result);
    static nlohmann::json SerializeMultipleCameras(
        const std::vector<CameraDetectionResult>& results
    );
};

class NetworkStreamer {
public:
    explicit NetworkStreamer(const NetworkConfig& config);
    ~NetworkStreamer();
    
    bool Start();
    void Stop();
    bool IsRunning() const;
    
    // Send fused pose result
    void SendFusedPose(const FusedPoseResult& result);
    
    // Send individual camera detection
    void SendCameraDetection(const CameraDetectionResult& result);
    
    // Send multiple camera detections
    void SendMultipleCameras(const std::vector<CameraDetectionResult>& results);

private:
    NetworkConfig config;
    SocketType socket_;
    std::atomic<bool> running;
    
    std::thread senderThread;
    std::mutex queueMutex;
    std::queue<std::string> messageQueue;
    
    bool InitializeSocket();
    void CloseSocket();
    void SenderLoop();
    void SendMessage(const std::string& message);
};