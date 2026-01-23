#include "mjpegStreamer.hpp"
#include "logger.hpp"
#include <cstring>
#include <sstream>

// MJPEGStream Implementation
MJPEGStream::MJPEGStream(const std::string& streamName, int port)
    : streamName(streamName)
    , port(port)
    , running(false)
    , serverSocket(-1)
    , hasFrame(false) {}

MJPEGStream::~MJPEGStream() {
    Stop();
}

bool MJPEGStream::Start() {
    if (running) return true;
    
#ifdef _WIN32
    WSADATA wsaData;
    WSAStartup(MAKEWORD(2, 2), &wsaData);
#endif
    
    serverSocket = socket(AF_INET, SOCK_STREAM, 0);
    if (serverSocket < 0) {
        LOG_ERROR("Failed to create MJPEG socket for " + streamName);
        return false;
    }
    
    int opt = 1;
    setsockopt(serverSocket, SOL_SOCKET, SO_REUSEADDR, (char*)&opt, sizeof(opt));
    
    struct sockaddr_in serverAddr;
    std::memset(&serverAddr, 0, sizeof(serverAddr));
    serverAddr.sin_family = AF_INET;
    serverAddr.sin_addr.s_addr = INADDR_ANY;
    serverAddr.sin_port = htons(port);
    
    if (bind(serverSocket, (struct sockaddr*)&serverAddr, sizeof(serverAddr)) < 0) {
        LOG_ERROR("Failed to bind MJPEG socket on port " + std::to_string(port));
#ifdef _WIN32
        closesocket(serverSocket);
#else
        close(serverSocket);
#endif
        return false;
    }
    
    listen(serverSocket, 5);
    
    running = true;
    serverThread = std::thread(&MJPEGStream::ServerLoop, this);
    
    LOG_INFO("MJPEG stream '" + streamName + "' started on port " + std::to_string(port));
    return true;
}

void MJPEGStream::Stop() {
    if (!running) return;
    
    running = false;
    
    if (serverSocket >= 0) {
#ifdef _WIN32
        closesocket(serverSocket);
        WSACleanup();
#else
        close(serverSocket);
#endif
        serverSocket = -1;
    }
    
    if (serverThread.joinable()) {
        serverThread.join();
    }
    
    LOG_INFO("MJPEG stream '" + streamName + "' stopped");
}

bool MJPEGStream::IsRunning() const {
    return running;
}

void MJPEGStream::UpdateFrame(const cv::Mat& frame) {
    if (frame.empty()) return;
    
    std::lock_guard<std::mutex> lock(frameMutex);
    currentFrame = frame.clone();
    hasFrame = true;
}

void MJPEGStream::ServerLoop() {
    while (running) {
        struct sockaddr_in clientAddr;
        socklen_t clientLen = sizeof(clientAddr);
        
        SocketType clientSocket = accept(serverSocket, (struct sockaddr*)&clientAddr, &clientLen);
        
        if (clientSocket < 0) {
            if (running) {
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
            continue;
        }
        
        LOG_DEBUG("MJPEG client connected to " + streamName);
        
        // Handle client in separate thread
        std::thread([this, clientSocket]() {
            HandleClient(clientSocket);
        }).detach();
    }
}

void MJPEGStream::HandleClient(SocketType clientSocket) {
    // Send HTTP header for MJPEG stream
    std::string header = 
        "HTTP/1.1 200 OK\r\n"
        "Content-Type: multipart/x-mixed-replace; boundary=frame\r\n"
        "Cache-Control: no-cache\r\n"
        "Connection: close\r\n"
        "\r\n";
    
    send(clientSocket, header.c_str(), header.length(), 0);
    
    while (running) {
        std::vector<uchar> jpeg;
        
        {
            std::lock_guard<std::mutex> lock(frameMutex);
            if (!hasFrame || currentFrame.empty()) {
                std::this_thread::sleep_for(std::chrono::milliseconds(33));
                continue;
            }
            
            // Encode frame to JPEG
            std::vector<int> params = {cv::IMWRITE_JPEG_QUALITY, 80};
            if (!cv::imencode(".jpg", currentFrame, jpeg, params)) {
                continue;
            }
        }
        
        // Send frame with MJPEG boundary
        std::ostringstream frameHeader;
        frameHeader << "--frame\r\n"
                   << "Content-Type: image/jpeg\r\n"
                   << "Content-Length: " << jpeg.size() << "\r\n"
                   << "\r\n";
        
        std::string headerStr = frameHeader.str();
        
        // Send header
        if (send(clientSocket, headerStr.c_str(), headerStr.length(), 0) < 0) {
            break;
        }
        
        // Send JPEG data
        if (send(clientSocket, (char*)jpeg.data(), jpeg.size(), 0) < 0) {
            break;
        }
        
        // Send boundary end
        if (send(clientSocket, "\r\n", 2, 0) < 0) {
            break;
        }
        
        // Limit to ~30 FPS
        std::this_thread::sleep_for(std::chrono::milliseconds(33));
    }
    
#ifdef _WIN32
    closesocket(clientSocket);
#else
    close(clientSocket);
#endif
    
    LOG_DEBUG("MJPEG client disconnected from " + streamName);
}

bool MJPEGStream::EncodeFrameToJPEG() {
    std::lock_guard<std::mutex> lock(frameMutex);
    
    if (!hasFrame || currentFrame.empty()) {
        return false;
    }
    
    std::vector<int> params = {cv::IMWRITE_JPEG_QUALITY, 80};
    return cv::imencode(".jpg", currentFrame, jpegBuffer, params);
}

// MJPEGStreamManager Implementation
MJPEGStreamManager::MJPEGStreamManager() : nextPort(8081) {}

MJPEGStreamManager::~MJPEGStreamManager() {
    StopAll();
}

void MJPEGStreamManager::AddStream(const std::string& streamName, int startPort) {
    std::lock_guard<std::mutex> lock(streamsMutex);
    
    int port = (startPort > 0) ? startPort : nextPort++;
    auto stream = std::make_unique<MJPEGStream>(streamName, port);
    streams.push_back(std::move(stream));
    
    LOG_INFO("Added MJPEG stream: " + streamName + " on port " + std::to_string(port));
}

void MJPEGStreamManager::UpdateFrame(const std::string& streamName, const cv::Mat& frame) {
    std::lock_guard<std::mutex> lock(streamsMutex);
    
    for (auto& stream : streams) {
        if (stream->GetStreamName() == streamName) {
            stream->UpdateFrame(frame);
            return;
        }
    }
}

void MJPEGStreamManager::StartAll() {
    std::lock_guard<std::mutex> lock(streamsMutex);
    
    for (auto& stream : streams) {
        stream->Start();
    }
}

void MJPEGStreamManager::StopAll() {
    std::lock_guard<std::mutex> lock(streamsMutex);
    
    for (auto& stream : streams) {
        stream->Stop();
    }
}

std::vector<std::pair<std::string, int>> MJPEGStreamManager::GetStreamInfo() const {
    std::lock_guard<std::mutex> lock(streamsMutex);
    
    std::vector<std::pair<std::string, int>> info;
    for (const auto& stream : streams) {
        info.push_back({stream->GetStreamName(), stream->GetPort()});
    }
    
    return info;
}