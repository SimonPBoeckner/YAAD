#include "visualizationUtils.hpp"

void VisualizationUtils::DrawDetections(cv::Mat& frame, zarray_t* detections) {
    if (!detections) return;
    
    for (int i = 0; i < zarray_size(detections); i++) {
        apriltag_detection_t* det;
        zarray_get(detections, i, &det);
        if (!det) continue;

        // Draw corners with lines
        for (int j = 0; j < 4; j++) {
            int next = (j + 1) % 4;
            cv::line(
                frame,
                cv::Point(det->p[j][0], det->p[j][1]),
                cv::Point(det->p[next][0], det->p[next][1]),
                cv::Scalar(0, 255, 0),
                2
            );
        }

        // Draw center point
        cv::circle(
            frame, 
            cv::Point(det->c[0], det->c[1]), 
            5, 
            cv::Scalar(0, 0, 255), 
            -1
        );
        
        // Draw tag ID
        cv::putText(
            frame,
            "ID: " + std::to_string(det->id),
            cv::Point(det->c[0] + 10, det->c[1] - 10),
            cv::FONT_HERSHEY_SIMPLEX,
            0.7,
            cv::Scalar(255, 255, 0),
            2
        );
    }
}

void VisualizationUtils::DrawPoseOverlay(
    cv::Mat& frame, 
    const CameraPoseObject& poseData,
    const std::string& cameraName) {
    
    if (!poseData.isValid()) return;
    
    int y = 30;
    int lineHeight = 25;
    
    // Camera name
    cv::putText(
        frame,
        cameraName,
        cv::Point(10, y),
        cv::FONT_HERSHEY_SIMPLEX,
        0.6,
        cv::Scalar(255, 255, 255),
        2
    );
    y += lineHeight;
    
    // Tag IDs
    std::string tagIds = "Tags: ";
    for (size_t i = 0; i < poseData.tag_ids.size(); i++) {
        tagIds += std::to_string(poseData.tag_ids[i]);
        if (i < poseData.tag_ids.size() - 1) tagIds += ", ";
    }
    cv::putText(
        frame,
        tagIds,
        cv::Point(10, y),
        cv::FONT_HERSHEY_SIMPLEX,
        0.5,
        cv::Scalar(200, 200, 200),
        1
    );
    y += lineHeight;
    
    // Position
    char posText[100];
    snprintf(posText, sizeof(posText), "Pos: %.2f, %.2f, %.2f",
        poseData.pose_0.Translation().X().value(),
        poseData.pose_0.Translation().Y().value(),
        poseData.pose_0.Translation().Z().value()
    );
    cv::putText(
        frame,
        posText,
        cv::Point(10, y),
        cv::FONT_HERSHEY_SIMPLEX,
        0.5,
        cv::Scalar(200, 200, 200),
        1
    );
    y += lineHeight;
    
    // Error
    char errorText[100];
    snprintf(errorText, sizeof(errorText), "Error: %.4f", poseData.error_0);
    cv::putText(
        frame,
        errorText,
        cv::Point(10, y),
        cv::FONT_HERSHEY_SIMPLEX,
        0.5,
        cv::Scalar(200, 200, 200),
        1
    );
}

void VisualizationUtils::DrawFPS(cv::Mat& frame, double fps) {
    char fpsText[50];
    snprintf(fpsText, sizeof(fpsText), "FPS: %.1f", fps);
    
    cv::putText(
        frame,
        fpsText,
        cv::Point(frame.cols - 120, 30),
        cv::FONT_HERSHEY_SIMPLEX,
        0.7,
        cv::Scalar(0, 255, 0),
        2
    );
}

void VisualizationUtils::DrawConfidence(cv::Mat& frame, double confidence) {
    int barWidth = 200;
    int barHeight = 20;
    int x = frame.cols - barWidth - 20;
    int y = 50;
    
    // Background
    cv::rectangle(
        frame,
        cv::Point(x, y),
        cv::Point(x + barWidth, y + barHeight),
        cv::Scalar(50, 50, 50),
        -1
    );
    
    // Confidence bar
    int fillWidth = static_cast<int>(barWidth * confidence);
    cv::Scalar color = GetColorForConfidence(confidence);
    cv::rectangle(
        frame,
        cv::Point(x, y),
        cv::Point(x + fillWidth, y + barHeight),
        color,
        -1
    );
    
    // Border
    cv::rectangle(
        frame,
        cv::Point(x, y),
        cv::Point(x + barWidth, y + barHeight),
        cv::Scalar(200, 200, 200),
        1
    );
    
    // Text
    char confText[50];
    snprintf(confText, sizeof(confText), "Conf: %.1f%%", confidence * 100);
    cv::putText(
        frame,
        confText,
        cv::Point(x, y - 5),
        cv::FONT_HERSHEY_SIMPLEX,
        0.5,
        cv::Scalar(255, 255, 255),
        1
    );
}

cv::Scalar VisualizationUtils::GetColorForConfidence(double confidence) {
    if (confidence > 0.7) {
        return cv::Scalar(0, 255, 0);  // Green
    } else if (confidence > 0.4) {
        return cv::Scalar(0, 165, 255); // Orange
    } else {
        return cv::Scalar(0, 0, 255);   // Red
    }
}