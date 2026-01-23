#pragma once
#include <opencv2/opencv.hpp>
#include <string>
#include <thread>
#include <atomic>
#include <mutex>
#include <vector>
#include <memory>

#ifdef _WIN32
    #include <winsock2.h>
    #include <ws2tcpip.h>
    typedef SOCKET SocketType;
#else
    #include <sys/socket.h>
    #include <netinet/in.h>
    #include <arpa/inet.h>
    #include <unistd.h>
    typedef int SocketType;
#endif

// MJPEG streamer for a single camera
class MJPEGStream {
public:
    explicit MJPEGStream(const std::string& streamName, int port);
    ~MJPEGStream();
    
    bool Start();
    void Stop();
    bool IsRunning() const;
    
    // Update the frame to be streamed
    void UpdateFrame(const cv::Mat& frame);
    
    const std::string& GetStreamName() const { return streamName; }
    int GetPort() const { return port; }

private:
    std::string streamName;
    int port;
    std::atomic<bool> running;
    
    std::thread serverThread;
    SocketType serverSocket;
    
    mutable std::mutex frameMutex;
    cv::Mat currentFrame;
    std::vector<uchar> jpegBuffer;
    bool hasFrame;
    
    void ServerLoop();
    void HandleClient(SocketType clientSocket);
    bool EncodeFrameToJPEG();
};

// Manager for multiple MJPEG streams
class MJPEGStreamManager {
public:
    MJPEGStreamManager();
    ~MJPEGStreamManager();
    
    // Add a new stream
    void AddStream(const std::string& streamName, int startPort);
    
    // Update frame for a specific stream
    void UpdateFrame(const std::string& streamName, const cv::Mat& frame);
    
    // Start all streams
    void StartAll();
    
    // Stop all streams
    void StopAll();
    
    // Get stream info
    std::vector<std::pair<std::string, int>> GetStreamInfo() const;

private:
    std::vector<std::unique_ptr<MJPEGStream>> streams;
    mutable std::mutex streamsMutex;
    int nextPort;
};