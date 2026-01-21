#include "logger.hpp"
#include <sstream>
#include <chrono>
#include <iomanip>

Logger& Logger::Instance() {
    static Logger instance;
    return instance;
}

Logger::Logger() : minLevel(LogLevel::INFO) {}

Logger::~Logger() {
    if (logFile.is_open()) {
        logFile.close();
    }
}

void Logger::SetLevel(LogLevel level) {
    minLevel = level;
}

void Logger::SetLogFile(const std::string& filename) {
    std::lock_guard<std::mutex> lock(mutex);
    if (logFile.is_open()) {
        logFile.close();
    }
    logFile.open(filename, std::ios::app);
}

void Logger::Log(LogLevel level, const std::string& message) {
    if (level < minLevel) return;
    
    std::lock_guard<std::mutex> lock(mutex);
    std::string timestamp = GetTimestamp();
    std::string levelStr = LevelToString(level);
    
    std::ostringstream oss;
    oss << "[" << timestamp << "] [" << levelStr << "] " << message;
    
    std::cout << oss.str() << std::endl;
    
    if (logFile.is_open()) {
        logFile << oss.str() << std::endl;
        logFile.flush();
    }
}

void Logger::Debug(const std::string& msg) {
    Log(LogLevel::DEBUG, msg);
}

void Logger::Info(const std::string& msg) {
    Log(LogLevel::INFO, msg);
}

void Logger::Warning(const std::string& msg) {
    Log(LogLevel::WARNING, msg);
}

void Logger::Error(const std::string& msg) {
    Log(LogLevel::ERROR, msg);
}

void Logger::Critical(const std::string& msg) {
    Log(LogLevel::CRITICAL, msg);
}

std::string Logger::GetTimestamp() {
    auto now = std::chrono::system_clock::now();
    auto time = std::chrono::system_clock::to_time_t(now);
    std::stringstream ss;
    ss << std::put_time(std::localtime(&time), "%Y-%m-%d %H:%M:%S");
    return ss.str();
}

std::string Logger::LevelToString(LogLevel level) {
    switch (level) {
        case LogLevel::DEBUG: return "DEBUG";
        case LogLevel::INFO: return "INFO";
        case LogLevel::WARNING: return "WARNING";
        case LogLevel::ERROR: return "ERROR";
        case LogLevel::CRITICAL: return "CRITICAL";
        default: return "UNKNOWN";
    }
}