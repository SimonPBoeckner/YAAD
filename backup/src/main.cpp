#include <iostream>
#include <capture.hpp>
#include <fiducialDetector.hpp>
#include <cameraPoseEstimator.hpp>

using namespace std;

int main() {
    DefaultCapture camera;
    FiducialDetector detector;
    MultiTagCameraPoseEstimator poseEstimator;

    for (;;) {
        auto frame = camera.GetFrame();
        if (frame.empty()) {
            continue;
        }
        auto detections = detector.DetectFiducials(frame);
        auto pose = poseEstimator.SolveCameraPose(detections);

        int n = zarray_size(detections);

        for (int i = 0; i < n; i++) {
            apriltag_detection_t* det;
            zarray_get(detections, i, &det);

            printf("Pose estimation result:\n");
            printf("Tag ID: %d\n", det->id);
            printf("Center: (%f, %f)\n", det->c[0], det->c[1]);
            printf("Pose: X: %f m, Y: %f m, Z: %f m\n",
                   pose.pose_0.Translation().X().to<double>(),
                   pose.pose_0.Translation().Y().to<double>(),
                   pose.pose_0.Translation().Z().to<double>());
            printf("Error: %f\n", pose.error_0);
        }
        imshow("Camera", frame);
        if (cv::waitKey(30) >= 0) break;
    }

    return 0;
}