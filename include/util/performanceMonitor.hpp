#pragma once
#include <chrono>
#include <string>
#include <unordered_map>

class PerformanceMonitor {
public:
    static PerformanceMonitor& Instance();
    
    void StartTimer(const std::string& name);
    double StopTimer(const std::string& name);
    void PrintStats();
    void Reset();

private:
    PerformanceMonitor() = default;
    
    // Delete copy/move
    PerformanceMonitor(const PerformanceMonitor&) = delete;
    PerformanceMonitor& operator=(const PerformanceMonitor&) = delete;
    
    std::unordered_map<std::string, std::chrono::high_resolution_clock::time_point> timers;
    std::unordered_map<std::string, double> averages;
    std::unordered_map<std::string, size_t> counts;
};

// RAII timer for automatic timing
class ScopedTimer {
public:
    explicit ScopedTimer(const std::string& name);
    ~ScopedTimer();

private:
    std::string name;
};

// FPS Counter
class FPSCounter {
public:
    FPSCounter();
    void Tick();
    double GetFPS() const;

private:
    size_t frameCount;
    double fps;
    std::chrono::high_resolution_clock::time_point lastTime;
};

#define PERF_TIMER(name) ScopedTimer _timer_##__LINE__(name)