#include "performanceMonitor.hpp"
#include <iostream>

// PerformanceMonitor Implementation
PerformanceMonitor& PerformanceMonitor::Instance() {
    static PerformanceMonitor instance;
    return instance;
}

void PerformanceMonitor::StartTimer(const std::string& name) {
    timers[name] = std::chrono::high_resolution_clock::now();
}

double PerformanceMonitor::StopTimer(const std::string& name) {
    auto it = timers.find(name);
    if (it == timers.end()) {
        return -1.0;
    }
    
    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(
        end - it->second
    );
    
    double ms = duration.count() / 1000.0;
    
    // Update running average
    if (averages.find(name) == averages.end()) {
        averages[name] = ms;
        counts[name] = 1;
    } else {
        counts[name]++;
        averages[name] = (averages[name] * (counts[name] - 1) + ms) / counts[name];
    }
    
    return ms;
}

void PerformanceMonitor::PrintStats() {
    std::cout << "\n=== Performance Statistics ===\n";
    for (const auto& [name, avg] : averages) {
        std::cout << name << ": " << avg << " ms (avg over " 
                  << counts[name] << " samples)\n";
    }
    std::cout << "==============================\n\n";
}

void PerformanceMonitor::Reset() {
    timers.clear();
    averages.clear();
    counts.clear();
}

// ScopedTimer Implementation
ScopedTimer::ScopedTimer(const std::string& name) : name(name) {
    PerformanceMonitor::Instance().StartTimer(name);
}

ScopedTimer::~ScopedTimer() {
    PerformanceMonitor::Instance().StopTimer(name);
}

// FPSCounter Implementation
FPSCounter::FPSCounter() : frameCount(0), fps(0.0) {
    lastTime = std::chrono::high_resolution_clock::now();
}

void FPSCounter::Tick() {
    frameCount++;
    auto currentTime = std::chrono::high_resolution_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
        currentTime - lastTime
    ).count();
    
    if (elapsed >= 1000) {
        fps = frameCount * 1000.0 / elapsed;
        frameCount = 0;
        lastTime = currentTime;
    }
}

double FPSCounter::GetFPS() const {
    return fps;
}