#include "multiCameraManager.hpp"
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
    poseEstimator = std::make_unique<MultiTagCameraPoseEstimator>(
        config.cameraConfig,
        fieldLayout
    );
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
        result.hasFrame = true;
        result.frame = frame.clone();
        
        if (detections && zarray_size(detections.get()) > 0) {
            result.poseData = poseEstimator->SolveCameraPose(detections.get());
        }
        
        // Update latest result (thread-safe)
        {
            std::lock_guard<std::mutex> lock(resultMutex);
            latestResult = std::move(result);
            hasResult = true;
        }
    }
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