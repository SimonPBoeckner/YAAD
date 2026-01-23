#pragma once
#include <iostream>
#include <fstream>
#include <string>
#include <mutex>

enum class LogLevel {
    DEBUG,
    INFO,
    WARNING,
    ERROR,
    CRITICAL
};

class Logger {
public:
    static Logger& Instance();
    
    void SetLevel(LogLevel level);
    void SetLogFile(const std::string& filename);
    void Log(LogLevel level, const std::string& message);
    
    // Convenience methods
    void Debug(const std::string& msg);
    void Info(const std::string& msg);
    void Warning(const std::string& msg);
    void Error(const std::string& msg);
    void Critical(const std::string& msg);

private:
    Logger();
    ~Logger();
    
    // Delete copy/move
    Logger(const Logger&) = delete;
    Logger& operator=(const Logger&) = delete;
    
    std::string GetTimestamp();
    std::string LevelToString(LogLevel level);
    
    LogLevel minLevel;
    std::ofstream logFile;
    std::mutex mutex;
};

// Macros for easy logging
#define LOG_DEBUG(msg) Logger::Instance().Debug(msg)
#define LOG_INFO(msg) Logger::Instance().Info(msg)
#define LOG_WARNING(msg) Logger::Instance().Warning(msg)
#define LOG_ERROR(msg) Logger::Instance().Error(msg)
#define LOG_CRITICAL(msg) Logger::Instance().Critical(msg)