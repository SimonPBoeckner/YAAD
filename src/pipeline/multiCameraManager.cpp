#include "multiCameraManager.hpp"
#include "visualizationUtils.hpp"
#include "logger.hpp"

// CameraStream Implementation
CameraStream::CameraStream(
    const CameraStreamConfig& config,
    std::shared_ptr<FieldLayout> layout)
    : config(config)
    , fieldLayout(layout)
    , running(false)
    , hasResult(false) {
    
    capture = std::make_unique<DefaultCapture>();
    detector = std::make_unique<FiducialDetector>(config.detectorConfig);
    
    if (config.enablePoseEstimation) {
        poseEstimator = std::make_unique<MultiTagCameraPoseEstimator>(
            config.cameraConfig,
            fieldLayout
        );
    }
    
    if (config.enableAngleCalculation) {
        angleCalculator = std::make_unique<CameraMatrixTagAngleCalculator>(
            config.cameraConfig
        );
    }
}

CameraStream::~CameraStream() {
    Stop();
}

void CameraStream::Start() {
    if (running) return;
    
    if (!capture->Init(config.cameraIndex, config.captureConfig)) {
        LOG_ERROR("Failed to initialize camera: " + config.cameraName);
        return;
    }
    
    running = true;
    workerThread = std::thread(&CameraStream::ProcessingLoop, this);
    LOG_INFO("Started camera stream: " + config.cameraName);
}

void CameraStream::Stop() {
    if (!running) return;
    
    running = false;
    if (workerThread.joinable()) {
        workerThread.join();
    }
    LOG_INFO("Stopped camera stream: " + config.cameraName);
}

bool CameraStream::IsRunning() const {
    return running;
}

std::optional<CameraDetectionResult> CameraStream::GetLatestResult() {
    std::lock_guard<std::mutex> lock(resultMutex);
    if (!hasResult) {
        return std::nullopt;
    }
    return latestResult;
}

void CameraStream::ProcessingLoop() {
    FPSCounter fpsCounter;
    
    while (running) {
        auto frameOpt = capture->GetFrame();
        if (!frameOpt) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            continue;
        }
        
        cv::Mat frame = *frameOpt;
        ZArrayPtr detections = detector->DetectFiducials(frame);
        
        CameraDetectionResult result;
        result.cameraName = config.cameraName;
        result.cameraIndex = config.cameraIndex;
        result.timestamp = std::chrono::system_clock::now();
        result.hasPose = false;
        result.hasAngle = false;
        result.hasFrame = false;
        
        if (detections && zarray_size(detections.get()) > 0) {
            // Pose estimation
            if (config.enablePoseEstimation && poseEstimator) {
                result.poseData = poseEstimator->SolveCameraPose(detections.get());
                result.hasPose = result.poseData.isValid();
                
                if (result.hasPose && config.storeFrames) {
                    VisualizationUtils::DrawDetections(frame, detections.get());
                    VisualizationUtils::DrawPoseOverlay(frame, result.poseData, config.cameraName);
                }
            }
            
            // Angle calculation
            if (config.enableAngleCalculation && angleCalculator) {
                result.angleData = angleCalculator->CalculateTagAngle(detections.get());
                result.hasAngle = result.angleData.isValid();
                
                if (result.hasAngle && config.storeFrames) {
                    // Draw angle visualization
                    DrawAngleVisualization(frame, result.angleData);
                }
            }
        }
        
        // Only clone and store frame if needed for visualization/streaming
        if (config.storeFrames) {
            // Draw FPS
            fpsCounter.Tick();
            VisualizationUtils::DrawFPS(frame, fpsCounter.GetFPS());
            
            result.hasFrame = true;
            result.frame = frame.clone();
        } else {
            result.hasFrame = false;
            fpsCounter.Tick(); // Still track FPS even if not storing frames
        }
        
        // Update latest result (thread-safe)
        {
            std::lock_guard<std::mutex> lock(resultMutex);
            latestResult = std::move(result);
            hasResult = true;
        }
    }
}

void CameraStream::DrawAngleVisualization(cv::Mat& frame, const TagAngleObject& angleData) {
    // Draw tag ID and distance
    char text[100];
    snprintf(text, sizeof(text), "Tag %d - Dist: %.2fm", 
             angleData.tag_id, angleData.distance);
    cv::putText(frame, text, cv::Point(10, frame.rows - 140),
                cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 0), 2);
    
    // Draw pose information
    snprintf(text, sizeof(text), "Pose0: [%.2f, %.2f, %.2f] Err: %.4f",
             angleData.pose0.Translation().X().value(),
             angleData.pose0.Translation().Y().value(),
             angleData.pose0.Translation().Z().value(),
             angleData.error0);
    cv::putText(frame, text, cv::Point(10, frame.rows - 115),
                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 255), 1);
    
    snprintf(text, sizeof(text), "Pose1: [%.2f, %.2f, %.2f] Err: %.4f",
             angleData.pose1.Translation().X().value(),
             angleData.pose1.Translation().Y().value(),
             angleData.pose1.Translation().Z().value(),
             angleData.error1);
    cv::putText(frame, text, cv::Point(10, frame.rows - 95),
                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 255), 1);
    
    // Indicate which pose is better
    std::string bestPose = (angleData.error0 < angleData.error1) ? "Using Pose0" : "Using Pose1";
    cv::putText(frame, bestPose, cv::Point(10, frame.rows - 75),
                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 1);
    
    // Draw corner angles (compact format)
    snprintf(text, sizeof(text), "Corners: [%.1f,%.1f] [%.1f,%.1f] [%.1f,%.1f] [%.1f,%.1f]", 
             angleData.corners(0, 0) * 180.0 / M_PI, angleData.corners(0, 1) * 180.0 / M_PI,
             angleData.corners(1, 0) * 180.0 / M_PI, angleData.corners(1, 1) * 180.0 / M_PI,
             angleData.corners(2, 0) * 180.0 / M_PI, angleData.corners(2, 1) * 180.0 / M_PI,
             angleData.corners(3, 0) * 180.0 / M_PI, angleData.corners(3, 1) * 180.0 / M_PI);
    cv::putText(frame, text, cv::Point(10, frame.rows - 50),
                cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(200, 200, 255), 1);
}

// MultiCameraManager Implementation
MultiCameraManager::MultiCameraManager(std::shared_ptr<FieldLayout> layout)
    : fieldLayout(layout) {}

MultiCameraManager::~MultiCameraManager() {
    StopAll();
}

void MultiCameraManager::AddCamera(const CameraStreamConfig& config) {
    std::lock_guard<std::mutex> lock(camerasMutex);
    auto stream = std::make_unique<CameraStream>(config, fieldLayout);
    cameras.push_back(std::move(stream));
    LOG_INFO("Added camera: " + config.cameraName);
}

void MultiCameraManager::StartAll() {
    std::lock_guard<std::mutex> lock(camerasMutex);
    for (auto& camera : cameras) {
        camera->Start();
    }
}

void MultiCameraManager::StopAll() {
    std::lock_guard<std::mutex> lock(camerasMutex);
    for (auto& camera : cameras) {
        camera->Stop();
    }
}

std::vector<CameraDetectionResult> MultiCameraManager::GetAllLatestResults() {
    std::lock_guard<std::mutex> lock(camerasMutex);
    std::vector<CameraDetectionResult> results;
    
    for (auto& camera : cameras) {
        auto result = camera->GetLatestResult();
        if (result) {
            results.push_back(*result);
        }
    }
    
    return results;
}

std::optional<CameraDetectionResult> MultiCameraManager::GetCameraResult(
    const std::string& cameraName) {
    std::lock_guard<std::mutex> lock(camerasMutex);
    
    for (auto& camera : cameras) {
        if (camera->GetName() == cameraName) {
            return camera->GetLatestResult();
        }
    }
    
    return std::nullopt;
}

size_t MultiCameraManager::GetCameraCount() const {
    std::lock_guard<std::mutex> lock(camerasMutex);
    return cameras.size();
}