#pragma once
#include <opencv2/opencv.hpp>
#include <optional>

struct CaptureConfig {
    int width = 640;
    int height = 480;
    int fps = 60;
    double brightness = -1;  // -1 means don't set
    double contrast = -1;
    double saturation = -1;
    double hue = -1;
    double gain = -1;
    double exposure = -1;
};

class Capture {
public:
    Capture() = default;
    virtual ~Capture() = default;
    
    // Delete copy/move to prevent issues with cv::VideoCapture
    Capture(const Capture&) = delete;
    Capture& operator=(const Capture&) = delete;
    Capture(Capture&&) = delete;
    Capture& operator=(Capture&&) = delete;

    virtual bool Init(int camIndex = 0, const CaptureConfig& config = CaptureConfig{}) = 0;
    virtual std::optional<cv::Mat> GetFrame() = 0;
    virtual bool IsOpened() const = 0;
};

class DefaultCapture : public Capture {
public:
    DefaultCapture() = default;
    ~DefaultCapture() override;
    
    bool Init(int camIndex = 0, const CaptureConfig& config = CaptureConfig{}) override;
    std::optional<cv::Mat> GetFrame() override;
    bool IsOpened() const override;

private:
    void ApplyConfig(const CaptureConfig& config);
    
    cv::VideoCapture cap;
    CaptureConfig currentConfig;
};