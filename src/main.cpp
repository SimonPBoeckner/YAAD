#include <iostream>
#include <capture.hpp>
#include <fiducialDetector.hpp>

using namespace std;

int main() {
    DefaultCapture camera;
    FiducialDetector detector;

    for (;;) {
        auto frame = camera.GetFrame();
        if (frame.empty()) {
            continue;
        }
        auto detections = detector.DetectFiducials(frame);

        int n = zarray_size(detections);

        for (int i = 0; i < n; i++) {
            apriltag_detection_t* det;
            zarray_get(detections, i, &det);

            printf("Tag ID: %d\n", det->id);
            printf("Center: (%f, %f)\n", det->c[0], det->c[1]);
        }
        imshow("Camera", frame);
        if (cv::waitKey(30) >= 0) break;
    }

    return 0;
}