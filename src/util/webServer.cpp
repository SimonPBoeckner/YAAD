#include "webServer.hpp"
#include "logger.hpp"
#include "networkStreamer.hpp"
#include <sstream>
#include <cstring>

#ifdef _WIN32
    #include <winsock2.h>
    #include <ws2tcpip.h>
#else
    #include <sys/socket.h>
    #include <netinet/in.h>
    #include <arpa/inet.h>
    #include <unistd.h>
#endif

WebServer::WebServer(int port)
    : port(port)
    , running(false) {}

WebServer::~WebServer() {
    Stop();
}

bool WebServer::Start() {
    if (running) return true;
    
    running = true;
    serverThread = std::thread(&WebServer::ServerLoop, this);
    
    LOG_INFO("Web server started on port " + std::to_string(port));
    return true;
}

void WebServer::Stop() {
    if (!running) return;
    
    running = false;
    if (serverThread.joinable()) {
        serverThread.join();
    }
    
    LOG_INFO("Web server stopped");
}

bool WebServer::IsRunning() const {
    return running;
}

void WebServer::SetFusedPoseCallback(std::function<FusedPoseResult()> callback) {
    fusedPoseCallback = callback;
}

void WebServer::SetCameraResultsCallback(
    std::function<std::vector<CameraDetectionResult>()> callback) {
    cameraResultsCallback = callback;
}

void WebServer::ServerLoop() {
#ifdef _WIN32
    WSADATA wsaData;
    WSAStartup(MAKEWORD(2, 2), &wsaData);
#endif
    
    int serverSocket = socket(AF_INET, SOCK_STREAM, 0);
    if (serverSocket < 0) {
        LOG_ERROR("Failed to create server socket");
        return;
    }
    
    int opt = 1;
    setsockopt(serverSocket, SOL_SOCKET, SO_REUSEADDR, (char*)&opt, sizeof(opt));
    
    struct sockaddr_in serverAddr;
    std::memset(&serverAddr, 0, sizeof(serverAddr));
    serverAddr.sin_family = AF_INET;
    serverAddr.sin_addr.s_addr = INADDR_ANY;
    serverAddr.sin_port = htons(port);
    
    if (bind(serverSocket, (struct sockaddr*)&serverAddr, sizeof(serverAddr)) < 0) {
        LOG_ERROR("Failed to bind server socket");
#ifdef _WIN32
        closesocket(serverSocket);
#else
        close(serverSocket);
#endif
        return;
    }
    
    listen(serverSocket, 5);
    
    while (running) {
        struct sockaddr_in clientAddr;
        socklen_t clientLen = sizeof(clientAddr);
        
#ifdef _WIN32
        SOCKET clientSocket = accept(serverSocket, (struct sockaddr*)&clientAddr, &clientLen);
#else
        int clientSocket = accept(serverSocket, (struct sockaddr*)&clientAddr, &clientLen);
#endif
        
        if (clientSocket < 0) {
            continue;
        }
        
        char buffer[4096];
        int bytesRead = recv(clientSocket, buffer, sizeof(buffer) - 1, 0);
        
        if (bytesRead > 0) {
            buffer[bytesRead] = '\0';
            std::string request(buffer);
            std::string response = HandleRequest(request);
            
            send(clientSocket, response.c_str(), response.length(), 0);
        }
        
#ifdef _WIN32
        closesocket(clientSocket);
#else
        close(clientSocket);
#endif
    }
    
#ifdef _WIN32
    closesocket(serverSocket);
    WSACleanup();
#else
    close(serverSocket);
#endif
}

std::string WebServer::HandleRequest(const std::string& request) {
    std::istringstream iss(request);
    std::string method, path, version;
    iss >> method >> path >> version;
    
    if (path == "/" || path == "/index.html") {
        return HandleIndex();
    } else if (path == "/api/status") {
        return HandleAPIStatus();
    } else if (path == "/api/fused_pose") {
        return HandleAPIFusedPose();
    } else if (path == "/api/cameras") {
        return HandleAPICameras();
    }
    
    std::string response = "HTTP/1.1 404 Not Found\r\nContent-Type: text/plain\r\n\r\n404 Not Found";
    return response;
}

std::string WebServer::HandleIndex() {
    std::string html = GenerateIndexPage();
    std::string response = "HTTP/1.1 200 OK\r\nContent-Type: text/html\r\n\r\n" + html;
    return response;
}

std::string WebServer::HandleAPIStatus() {
    nlohmann::json j;
    j["status"] = "running";
    j["timestamp"] = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::system_clock::now().time_since_epoch()
    ).count();
    
    std::string body = j.dump();
    std::string response = "HTTP/1.1 200 OK\r\nContent-Type: application/json\r\n\r\n" + body;
    return response;
}

std::string WebServer::HandleAPIFusedPose() {
    if (fusedPoseCallback) {
        FusedPoseResult result = fusedPoseCallback();
        nlohmann::json j = DetectionSerializer::SerializeFusedPose(result);
        std::string body = j.dump();
        std::string response = "HTTP/1.1 200 OK\r\nContent-Type: application/json\r\n\r\n" + body;
        return response;
    }
    
    std::string response = "HTTP/1.1 503 Service Unavailable\r\n\r\n";
    return response;
}

std::string WebServer::HandleAPICameras() {
    if (cameraResultsCallback) {
        auto results = cameraResultsCallback();
        nlohmann::json j = DetectionSerializer::SerializeMultipleCameras(results);
        std::string body = j.dump();
        std::string response = "HTTP/1.1 200 OK\r\nContent-Type: application/json\r\n\r\n" + body;
        return response;
    }
    
    std::string response = "HTTP/1.1 503 Service Unavailable\r\n\r\n";
    return response;
}

std::string WebServer::GenerateIndexPage() {
    return R"html(
<!DOCTYPE html>
<html>
<head>
    <title>AprilTag Vision System</title>
    <style>
        body { font-family: Arial, sans-serif; margin: 20px; background: #1a1a1a; color: #fff; }
        h1 { color: #4CAF50; }
        .container { max-width: 1200px; margin: 0 auto; }
        .card { background: #2a2a2a; border-radius: 8px; padding: 20px; margin: 20px 0; box-shadow: 0 2px 4px rgba(0,0,0,0.3); }
        .grid { display: grid; grid-template-columns: repeat(auto-fit, minmax(300px, 1fr)); gap: 20px; }
        .metric { font-size: 24px; font-weight: bold; color: #4CAF50; }
        .label { color: #888; font-size: 14px; }
        table { width: 100%; border-collapse: collapse; }
        th, td { padding: 10px; text-align: left; border-bottom: 1px solid #444; }
        th { background: #333; color: #4CAF50; }
        .confidence-bar { width: 100%; height: 20px; background: #333; border-radius: 4px; overflow: hidden; }
        .confidence-fill { height: 100%; background: linear-gradient(90deg, #f44336, #ff9800, #4CAF50); }
        #status { display: inline-block; width: 12px; height: 12px; border-radius: 50%; background: #4CAF50; }
        .position { font-family: monospace; }
    </style>
</head>
<body>
    <div class="container">
        <h1>🎯 AprilTag Vision System</h1>
        
        <div class="card">
            <h2>System Status <span id="status"></span></h2>
            <div class="grid">
                <div>
                    <div class="label">Cameras Active</div>
                    <div class="metric" id="camera-count">-</div>
                </div>
                <div>
                    <div class="label">Confidence</div>
                    <div class="metric" id="confidence">-</div>
                </div>
                <div>
                    <div class="label">Update Rate</div>
                    <div class="metric" id="update-rate">- Hz</div>
                </div>
            </div>
        </div>
        
        <div class="card">
            <h2>Fused Pose Estimate</h2>
            <table>
                <tr>
                    <th>Axis</th>
                    <th>Position (m)</th>
                    <th>Orientation (rad)</th>
                    <th>Velocity (m/s)</th>
                </tr>
                <tr>
                    <td>X</td>
                    <td class="position" id="pos-x">-</td>
                    <td class="position" id="rot-roll">-</td>
                    <td class="position" id="vel-x">-</td>
                </tr>
                <tr>
                    <td>Y</td>
                    <td class="position" id="pos-y">-</td>
                    <td class="position" id="rot-pitch">-</td>
                    <td class="position" id="vel-y">-</td>
                </tr>
                <tr>
                    <td>Z</td>
                    <td class="position" id="pos-z">-</td>
                    <td class="position" id="rot-yaw">-</td>
                    <td class="position" id="vel-z">-</td>
                </tr>
            </table>
        </div>
        
        <div class="card">
            <h2>Camera Feeds</h2>
            <div id="cameras"></div>
        </div>
    </div>
    
    <script>
        let updateCount = 0;
        let lastTime = Date.now();
        
        function updateData() {
            fetch('/api/fused_pose')
                .then(r => r.json())
                .then(data => {
                    document.getElementById('camera-count').textContent = data.num_cameras_used;
                    document.getElementById('confidence').textContent = (data.confidence * 100).toFixed(1) + '%';
                    
                    document.getElementById('pos-x').textContent = data.pose.position.x.toFixed(3);
                    document.getElementById('pos-y').textContent = data.pose.position.y.toFixed(3);
                    document.getElementById('pos-z').textContent = data.pose.position.z.toFixed(3);
                    
                    document.getElementById('rot-roll').textContent = data.pose.orientation.roll.toFixed(3);
                    document.getElementById('rot-pitch').textContent = data.pose.orientation.pitch.toFixed(3);
                    document.getElementById('rot-yaw').textContent = data.pose.orientation.yaw.toFixed(3);
                    
                    document.getElementById('vel-x').textContent = data.velocity.x.toFixed(3);
                    document.getElementById('vel-y').textContent = data.velocity.y.toFixed(3);
                    document.getElementById('vel-z').textContent = data.velocity.z.toFixed(3);
                    
                    updateCount++;
                    const now = Date.now();
                    if (now - lastTime >= 1000) {
                        document.getElementById('update-rate').textContent = updateCount.toFixed(1);
                        updateCount = 0;
                        lastTime = now;
                    }
                });
            
            fetch('/api/cameras')
                .then(r => r.json())
                .then(data => {
                    const container = document.getElementById('cameras');
                    container.innerHTML = data.cameras.map(cam => `
                        <div style="margin: 10px 0; padding: 10px; background: #333; border-radius: 4px;">
                            <strong>${cam.camera_name}</strong><br>
                            Tags: ${cam.tag_ids ? cam.tag_ids.join(', ') : 'None'}<br>
                            Error: ${cam.error ? cam.error.toFixed(4) : 'N/A'}
                        </div>
                    `).join('');
                });
        }
        
        setInterval(updateData, 100);
        updateData();
    </script>
</body>
</html>
)html";
}