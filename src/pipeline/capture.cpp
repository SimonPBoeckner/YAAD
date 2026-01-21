#include "capture.hpp"
#include <iostream>

DefaultCapture::~DefaultCapture() {
    if (cap.isOpened()) {
        cap.release();
    }
}

bool DefaultCapture::Init(int camIndex, const CaptureConfig& config) {
    if (cap.isOpened()) {
        return true;
    }

    if (!cap.open(camIndex)) {
        std::cerr << "Failed to open camera " << camIndex << "\n";
        return false;
    }

    currentConfig = config;
    ApplyConfig(config);
    
    return true;
}

void DefaultCapture::ApplyConfig(const CaptureConfig& config) {
    cap.set(cv::CAP_PROP_FRAME_WIDTH, config.width);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, config.height);
    cap.set(cv::CAP_PROP_FPS, config.fps);
    
    if (config.brightness >= 0) cap.set(cv::CAP_PROP_BRIGHTNESS, config.brightness);
    if (config.contrast >= 0) cap.set(cv::CAP_PROP_CONTRAST, config.contrast);
    if (config.saturation >= 0) cap.set(cv::CAP_PROP_SATURATION, config.saturation);
    if (config.hue >= 0) cap.set(cv::CAP_PROP_HUE, config.hue);
    if (config.gain >= 0) cap.set(cv::CAP_PROP_GAIN, config.gain);
    if (config.exposure >= 0) cap.set(cv::CAP_PROP_EXPOSURE, config.exposure);
}

std::optional<cv::Mat> DefaultCapture::GetFrame() {
    if (!cap.isOpened() && !Init(0, currentConfig)) {
        return std::nullopt;
    }

    cv::Mat frame;
    if (!cap.read(frame) || frame.empty()) {
        return std::nullopt;
    }

    return frame;
}

bool DefaultCapture::IsOpened() const {
    return cap.isOpened();
}