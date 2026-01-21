#include "capture.hpp"
#include <iostream>

Capture::Capture() {}

Capture::~Capture() {
    if (cap.isOpened()) {
        cap.release();
    }
}

bool Capture::Init(int camIndex)
{
    return false;
}

cv::Mat Capture::GetFrame()
{
    return cv::Mat();
}

bool DefaultCapture::Init(int camIndex) {
    if (cap.isOpened()) return true;

    if (!cap.open(camIndex)) {
        std::cerr << "Failed to open camera\n";
        return false;
    }

    cap.set(cv::CAP_PROP_FRAME_WIDTH, 640);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
    cap.set(cv::CAP_PROP_FPS, 60);
    cap.set(cv::CAP_PROP_BRIGHTNESS, 1);
    cap.set(cv::CAP_PROP_CONTRAST, 1);
    cap.set(cv::CAP_PROP_SATURATION, 1);
    cap.set(cv::CAP_PROP_HUE, 1);
    cap.set(cv::CAP_PROP_GAIN, 1);
    cap.set(cv::CAP_PROP_EXPOSURE, 1);

    return true;
}

cv::Mat DefaultCapture::GetFrame() {
    if (!cap.isOpened() && !Init()) {
        return cv::Mat();
    }

    if (!cap.read(frame) || frame.empty()) {
        return cv::Mat();
    }

    return frame;
}