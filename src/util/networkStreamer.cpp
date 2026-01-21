#include "networkStreamer.hpp"
#include "logger.hpp"
#include <cstring>

// DetectionSerializer Implementation
nlohmann::json DetectionSerializer::SerializeFusedPose(const FusedPoseResult& result) {
    nlohmann::json j;
    
    j["type"] = "fused_pose";
    j["timestamp"] = std::chrono::duration_cast<std::chrono::milliseconds>(
        result.timestamp.time_since_epoch()
    ).count();
    
    j["pose"]["position"]["x"] = result.pose.Translation().X().value();
    j["pose"]["position"]["y"] = result.pose.Translation().Y().value();
    j["pose"]["position"]["z"] = result.pose.Translation().Z().value();
    
    j["pose"]["orientation"]["roll"] = result.pose.Rotation().X().value();
    j["pose"]["orientation"]["pitch"] = result.pose.Rotation().Y().value();
    j["pose"]["orientation"]["yaw"] = result.pose.Rotation().Z().value();
    
    j["velocity"]["x"] = result.velocity.x();
    j["velocity"]["y"] = result.velocity.y();
    j["velocity"]["z"] = result.velocity.z();
    
    j["confidence"] = result.confidence;
    j["num_cameras_used"] = result.numCamerasUsed;
    j["contributing_cameras"] = result.contributingCameras;
    
    return j;
}

nlohmann::json DetectionSerializer::SerializeCameraDetection(
    const CameraDetectionResult& result) {
    
    nlohmann::json j;
    
    j["type"] = "camera_detection";
    j["camera_name"] = result.cameraName;
    j["camera_index"] = result.cameraIndex;
    j["timestamp"] = std::chrono::duration_cast<std::chrono::milliseconds>(
        result.timestamp.time_since_epoch()
    ).count();
    
    if (result.poseData.isValid()) {
        j["tag_ids"] = result.poseData.tag_ids;
        
        j["pose"]["position"]["x"] = result.poseData.pose_0.Translation().X().value();
        j["pose"]["position"]["y"] = result.poseData.pose_0.Translation().Y().value();
        j["pose"]["position"]["z"] = result.poseData.pose_0.Translation().Z().value();
        
        j["pose"]["orientation"]["roll"] = result.poseData.pose_0.Rotation().X().value();
        j["pose"]["orientation"]["pitch"] = result.poseData.pose_0.Rotation().Y().value();
        j["pose"]["orientation"]["yaw"] = result.poseData.pose_0.Rotation().Z().value();
        
        j["error"] = result.poseData.error_0;
    }
    
    return j;
}

nlohmann::json DetectionSerializer::SerializeMultipleCameras(
    const std::vector<CameraDetectionResult>& results) {
    
    nlohmann::json j;
    j["type"] = "multiple_cameras";
    j["count"] = results.size();
    
    nlohmann::json cameras = nlohmann::json::array();
    for (const auto& result : results) {
        cameras.push_back(SerializeCameraDetection(result));
    }
    j["cameras"] = cameras;
    
    return j;
}

// NetworkStreamer Implementation
NetworkStreamer::NetworkStreamer(const NetworkConfig& config)
    : config(config)
    , socket_(INVALID_SOCKET_VALUE)
    , running(false) {}

NetworkStreamer::~NetworkStreamer() {
    Stop();
}

bool NetworkStreamer::Start() {
    if (running) return true;
    
    if (!InitializeSocket()) {
        LOG_ERROR("Failed to initialize network socket");
        return false;
    }
    
    running = true;
    senderThread = std::thread(&NetworkStreamer::SenderLoop, this);
    
    LOG_INFO("Network streamer started on " + config.address + ":" + std::to_string(config.port));
    return true;
}

void NetworkStreamer::Stop() {
    if (!running) return;
    
    running = false;
    if (senderThread.joinable()) {
        senderThread.join();
    }
    
    CloseSocket();
    LOG_INFO("Network streamer stopped");
}

bool NetworkStreamer::IsRunning() const {
    return running;
}

void NetworkStreamer::SendFusedPose(const FusedPoseResult& result) {
    if (!running) return;
    
    nlohmann::json j = DetectionSerializer::SerializeFusedPose(result);
    std::string message = j.dump();
    
    std::lock_guard<std::mutex> lock(queueMutex);
    if (messageQueue.size() < static_cast<size_t>(config.maxQueueSize)) {
        messageQueue.push(message);
    } else {
        LOG_WARNING("Network queue full, dropping message");
    }
}

void NetworkStreamer::SendCameraDetection(const CameraDetectionResult& result) {
    if (!running) return;
    
    nlohmann::json j = DetectionSerializer::SerializeCameraDetection(result);
    std::string message = j.dump();
    
    std::lock_guard<std::mutex> lock(queueMutex);
    if (messageQueue.size() < static_cast<size_t>(config.maxQueueSize)) {
        messageQueue.push(message);
    }
}

void NetworkStreamer::SendMultipleCameras(
    const std::vector<CameraDetectionResult>& results) {
    if (!running) return;
    
    nlohmann::json j = DetectionSerializer::SerializeMultipleCameras(results);
    std::string message = j.dump();
    
    std::lock_guard<std::mutex> lock(queueMutex);
    if (messageQueue.size() < static_cast<size_t>(config.maxQueueSize)) {
        messageQueue.push(message);
    }
}

bool NetworkStreamer::InitializeSocket() {
#ifdef _WIN32
    WSADATA wsaData;
    if (WSAStartup(MAKEWORD(2, 2), &wsaData) != 0) {
        return false;
    }
#endif
    
    if (config.protocol == NetworkProtocol::UDP) {
        socket_ = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    } else {
        socket_ = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    }
    
    if (socket_ == INVALID_SOCKET_VALUE) {
        return false;
    }
    
    return true;
}

void NetworkStreamer::CloseSocket() {
    if (socket_ != INVALID_SOCKET_VALUE) {
#ifdef _WIN32
        closesocket(socket_);
        WSACleanup();
#else
        close(socket_);
#endif
        socket_ = INVALID_SOCKET_VALUE;
    }
}

void NetworkStreamer::SenderLoop() {
    struct sockaddr_in destAddr;
    std::memset(&destAddr, 0, sizeof(destAddr));
    destAddr.sin_family = AF_INET;
    destAddr.sin_port = htons(config.port);
    inet_pton(AF_INET, config.address.c_str(), &destAddr.sin_addr);
    
    while (running) {
        std::string message;
        
        {
            std::lock_guard<std::mutex> lock(queueMutex);
            if (!messageQueue.empty()) {
                message = messageQueue.front();
                messageQueue.pop();
            }
        }
        
        if (!message.empty()) {
            if (config.protocol == NetworkProtocol::UDP) {
                sendto(socket_, message.c_str(), message.length(), 0,
                       (struct sockaddr*)&destAddr, sizeof(destAddr));
            } else {
                // TCP would need connection management
                send(socket_, message.c_str(), message.length(), 0);
            }
        } else {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }
}

void NetworkStreamer::SendMessage(const std::string& message) {
    std::lock_guard<std::mutex> lock(queueMutex);
    if (messageQueue.size() < static_cast<size_t>(config.maxQueueSize)) {
        messageQueue.push(message);
    }
}