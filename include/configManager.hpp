#pragma once
#include <string>
#include <optional>

struct AppConfig {
    // Camera settings
    int cameraIndex = 0;
    int cameraWidth = 640;
    int cameraHeight = 480;
    int cameraFPS = 60;
    
    // Paths
    std::string fieldLayoutPath = "";
    std::string cameraCalibrationPath = "";
    std::string logFilePath = "apriltag.log";
    
    // Detection settings
    float quadDecimate = 1.0f;
    float quadSigma = 0.0f;
    int numThreads = 1;
    bool refineEdges = true;
    
    // Tag settings
    double tagSize = 0.162; // meters
    
    // Display settings
    bool showVisualization = true;
    bool showFPS = true;
    bool verboseLogging = false;
    
    // Performance settings
    bool enablePerformanceMonitoring = false;
    int performanceReportInterval = 100; // frames
};

class ConfigManager {
public:
    static std::optional<AppConfig> LoadFromFile(const std::string& filepath);
    static bool SaveToFile(const std::string& filepath, const AppConfig& config);
    static AppConfig CreateDefault();
};