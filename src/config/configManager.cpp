#include "configManager.hpp"
#include <nlohmann/json.hpp>
#include <fstream>
#include <iostream>

std::optional<AppConfig> ConfigManager::LoadFromFile(const std::string& filepath) {
    std::ifstream file(filepath);
    if (!file.is_open()) {
        std::cerr << "Failed to open config file: " << filepath << std::endl;
        return std::nullopt;
    }
    
    nlohmann::json j;
    try {
        file >> j;
    } catch (const nlohmann::json::exception& e) {
        std::cerr << "Failed to parse config file: " << e.what() << std::endl;
        return std::nullopt;
    }
    
    AppConfig config;
    
    // Camera settings
    if (j.contains("camera")) {
        config.cameraIndex = j["camera"].value("index", 0);
        config.cameraWidth = j["camera"].value("width", 640);
        config.cameraHeight = j["camera"].value("height", 480);
        config.cameraFPS = j["camera"].value("fps", 60);
    }
    
    // Paths
    if (j.contains("paths")) {
        config.fieldLayoutPath = j["paths"].value("field_layout", "");
        config.cameraCalibrationPath = j["paths"].value("camera_calibration", "");
        config.logFilePath = j["paths"].value("log_file", "apriltag.log");
    }
    
    // Detection settings
    if (j.contains("detection")) {
        config.quadDecimate = j["detection"].value("quad_decimate", 1.0f);
        config.quadSigma = j["detection"].value("quad_sigma", 0.0f);
        config.numThreads = j["detection"].value("num_threads", 1);
        config.refineEdges = j["detection"].value("refine_edges", true);
    }
    
    // Tag settings
    if (j.contains("tag")) {
        config.tagSize = j["tag"].value("size", 0.162);
    }
    
    // Display settings
    if (j.contains("display")) {
        config.showVisualization = j["display"].value("visualization", true);
        config.showFPS = j["display"].value("fps", true);
        config.verboseLogging = j["display"].value("verbose", false);
    }
    
    // Performance settings
    if (j.contains("performance")) {
        config.enablePerformanceMonitoring = j["performance"].value("enabled", false);
        config.performanceReportInterval = j["performance"].value("report_interval", 100);
    }
    
    return config;
}

bool ConfigManager::SaveToFile(const std::string& filepath, const AppConfig& config) {
    nlohmann::json j;
    
    j["camera"]["index"] = config.cameraIndex;
    j["camera"]["width"] = config.cameraWidth;
    j["camera"]["height"] = config.cameraHeight;
    j["camera"]["fps"] = config.cameraFPS;
    
    j["paths"]["field_layout"] = config.fieldLayoutPath;
    j["paths"]["camera_calibration"] = config.cameraCalibrationPath;
    j["paths"]["log_file"] = config.logFilePath;
    
    j["detection"]["quad_decimate"] = config.quadDecimate;
    j["detection"]["quad_sigma"] = config.quadSigma;
    j["detection"]["num_threads"] = config.numThreads;
    j["detection"]["refine_edges"] = config.refineEdges;
    
    j["tag"]["size"] = config.tagSize;
    
    j["display"]["visualization"] = config.showVisualization;
    j["display"]["fps"] = config.showFPS;
    j["display"]["verbose"] = config.verboseLogging;
    
    j["performance"]["enabled"] = config.enablePerformanceMonitoring;
    j["performance"]["report_interval"] = config.performanceReportInterval;
    
    std::ofstream file(filepath);
    if (!file.is_open()) {
        std::cerr << "Failed to create config file: " << filepath << std::endl;
        return false;
    }
    
    file << j.dump(4);
    return true;
}

AppConfig ConfigManager::CreateDefault() {
    return AppConfig{};
}