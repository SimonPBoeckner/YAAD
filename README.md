# AprilTag Multi-Camera Vision System

A production-ready, multi-camera AprilTag detection system with real-time pose estimation, sensor fusion, network streaming, and web-based monitoring.

## Features

- ✅ **Multi-Camera Support**: Thread-safe parallel processing of multiple cameras
- ✅ **Kalman Filtering**: Smooth pose estimation with velocity tracking
- ✅ **Sensor Fusion**: Weighted fusion of multiple camera measurements
- ✅ **Live Video Streaming**: MJPEG streams for each camera
- ✅ **Network Streaming**: UDP/TCP transmission of detection data
- ✅ **Web Interface**: Real-time dashboard with live feeds at `http://localhost:8080`
- ✅ **Performance Monitoring**: Built-in FPS tracking and performance metrics
- ✅ **Configurable**: JSON-based configuration system

## Architecture

```
┌─────────────┐     ┌─────────────┐     ┌─────────────┐
│  Camera 1   │────▶│  Detector   │────▶│   Fusion    │
└─────────────┘     └─────────────┘     └─────────────┘
                                               │
┌─────────────┐     ┌─────────────┐           │
│  Camera 2   │────▶│  Detector   │───────────┤
└─────────────┘     └─────────────┘           │
                                               ▼
┌─────────────┐     ┌─────────────┐     ┌─────────────┐
│  Camera N   │────▶│  Detector   │────▶│   Kalman    │
└─────────────┘     └─────────────┘     │   Filter    │
                                         └──────┬──────┘
                                                │
                    ┌───────────────────────────┼───────────────────┐
                    │                           │                   │
              ┌─────▼──────┐           ┌───────▼────────┐   ┌──────▼──────┐
              │   MJPEG    │           │    Network     │   │     Web     │
              │  Streams   │           │   Streaming    │   │  Interface  │
              └────────────┘           └────────────────┘   └─────────────┘
                Port 8081+                  Port 5800          Port 8080
```

## Installation

### Dependencies

```bash
# Ubuntu/Debian
sudo apt-get install \
    libopencv-dev \
    libeigen3-dev \
    nlohmann-json3-dev \
    apriltag-dev

# macOS
brew install opencv eigen nlohmann-json apriltag
```

### Compilation

```bash
g++ -std=c++17 \
    main_advanced.cpp \
    capture.cpp \
    fiducialDetector.cpp \
    coordinateChanger.cpp \
    poseEstimator.cpp \
    cameraPoseEstimator.cpp \
    tagAngleCalculator.cpp \
    kalmanFilter.cpp \
    cameraFusion.cpp \
    multiCameraManager.cpp \
    networkStreamer.cpp \
    mjpegStreamer.cpp \
    webServer.cpp \
    visualizationUtils.cpp \
    cameraCalibration.cpp \
    logger.cpp \
    performanceMonitor.cpp \
    configManager.cpp \
    -o vision_system \
    -lopencv_core -lopencv_imgproc -lopencv_calib3d -lopencv_imgcodecs \
    -lapriltag -lpthread
```

Or use CMake:

```cmake
cmake_minimum_required(VERSION 3.10)
project(AprilTagVision)

set(CMAKE_CXX_STANDARD 17)

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

add_executable(vision_system
    main_advanced.cpp
    # ... all other .cpp files
)

target_link_libraries(vision_system
    ${OpenCV_LIBS}
    Eigen3::Eigen
    apriltag
    pthread
)
```

## Configuration

Create `config_advanced.json`:

```json
{
  "camera": {
    "index": 0,
    "width": 640,
    "height": 480,
    "fps": 60
  },
  "paths": {
    "field_layout": "/path/to/field_layout.json",
    "camera_calibration": "/path/to/calibration.yml",
    "log_file": "apriltag.log"
  },
  "detection": {
    "quad_decimate": 1.0,
    "quad_sigma": 0.0,
    "num_threads": 4,
    "refine_edges": true
  },
  "tag": {
    "size": 0.162
  },
  "fusion": {
    "max_measurement_age": 0.5,
    "min_cameras_required": 1,
    "process_noise": 0.01
  },
  "network": {
    "enabled": true,
    "protocol": "UDP",
    "address": "127.0.0.1",
    "port": 5800
  }
}
```

## Usage

### Start the System

```bash
./vision_system config_advanced.json
```

### Access Web Interface

Open browser to `http://localhost:8080`

You'll see:
- Live camera feeds with AprilTag overlays
- Real-time fused pose estimate
- Position, orientation, and velocity
- Per-camera statistics
- System confidence metrics

### View Individual Camera Streams

Each camera has its own MJPEG stream:
- Camera 1: `http://localhost:8081`
- Camera 2: `http://localhost:8082`
- etc.

### Receive Network Data

Listen to UDP stream:
```bash
nc -ul 5800
```

Or use Python:
```python
import socket
import json

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(('127.0.0.1', 5800))

while True:
    data, addr = sock.recvfrom(4096)
    msg = json.loads(data.decode())
    print(f"Pose: {msg['pose']}")
    print(f"Confidence: {msg['confidence']}")
```

## API Endpoints

### HTTP (Port 8080)

- `GET /` - Web dashboard
- `GET /api/status` - System status
- `GET /api/fused_pose` - Current fused pose estimate
- `GET /api/cameras` - Individual camera detections
- `GET /api/streams` - List of MJPEG stream ports

### MJPEG Streams (Port 8081+)

- Each camera gets its own port starting at 8081
- Access via `http://localhost:PORT` in any browser/media player

### Network Stream (Port 5800 UDP)

JSON format:
```json
{
  "type": "fused_pose",
  "timestamp": 1234567890,
  "pose": {
    "position": {"x": 1.5, "y": 2.0, "z": 0.5},
    "orientation": {"roll": 0.0, "pitch": 0.1, "yaw": 1.57}
  },
  "velocity": {"x": 0.1, "y": 0.0, "z": 0.0},
  "confidence": 0.95,
  "num_cameras_used": 2
}
```

## Adding Multiple Cameras

In `main_advanced.cpp`:

```cpp
// Add camera 1
CameraStreamConfig cam1;
cam1.cameraIndex = 0;
cam1.cameraName = "front_camera";
// ... configure cam1
cameraManager.AddCamera(cam1);
mjpegManager.AddStream("front_camera", 8081);

// Add camera 2
CameraStreamConfig cam2;
cam2.cameraIndex = 1;
cam2.cameraName = "rear_camera";
// ... configure cam2
cameraManager.AddCamera(cam2);
mjpegManager.AddStream("rear_camera", 8082);
```

## Camera Calibration

Generate calibration file:

```bash
# Capture chessboard images
# Then run calibration (implement calibration tool or use OpenCV samples)

# Or load existing calibration
{
  "camera_matrix": [[fx, 0, cx], [0, fy, cy], [0, 0, 1]],
  "distortion_coefficients": [k1, k2, p1, p2, k3],
  "tag_size": 0.162
}
```

## Performance Tuning

### For High FPS:
- Reduce `quad_decimate` (faster but less accurate)
- Increase `num_threads`
- Lower camera resolution
- Disable visualization overlays

### For High Accuracy:
- Set `quad_decimate = 1.0`
- Enable `refine_edges`
- Use camera calibration
- Increase Kalman filter process noise

### For Multi-Camera Fusion:
- Set `min_cameras_required = 2` (requires 2+ cameras)
- Adjust `max_measurement_age` for camera sync
- Tune fusion weights based on camera quality

## Troubleshooting

### No video in web interface
- Check MJPEG streams are running: `curl http://localhost:8081`
- Verify camera permissions
- Check browser console for errors

### Poor pose estimation
- Calibrate cameras properly
- Ensure good lighting
- Check tag size is correct
- Verify field layout JSON is accurate

### High latency
- Reduce number of threads (paradoxically can help)
- Disable performance monitoring
- Optimize network queue size

### System crashes
- Check camera indices are correct
- Verify all dependencies installed
- Enable debug logging: `"verbose": true`

## License

[Your License Here]

## Credits

Built with:
- OpenCV - Computer vision
- Eigen - Linear algebra
- AprilTag - Fiducial detection
- nlohmann/json - JSON parsing