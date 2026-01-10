#include <capture.hpp>

int main() {
    DefaultCapture camera;

    for (;;) {
        if (camera.GetFrame().empty()) {
            continue;
        }
        imshow("Camera", camera.GetFrame());
        if (cv::waitKey(30) >= 0) break;
    }

    return 0;
}