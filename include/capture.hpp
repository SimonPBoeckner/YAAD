#pragma once
#include <opencv2/opencv.hpp>

class Capture {
public:
    Capture();
    virtual ~Capture();

    virtual bool Init(int camIndex = 0); // TODO: make this take a config object
    // virtual bool ConfigChanged(); // TODO: check if local config and remote config are different restarts capture to apply new settings
    virtual cv::Mat GetFrame();

protected:
    cv::VideoCapture cap;
    cv::Mat frame;
};

class DefaultCapture : public Capture {
public:
    bool Init(int camIndex = 1) override;
    cv::Mat GetFrame() override;
};
