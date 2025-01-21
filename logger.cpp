// logger.hpp

#include "logger.hpp"
#include <chrono>
#include <ctime>
#include <iomanip>

std::ofstream Logger::log_file;
std::mutex Logger::log_mutex;
const char* Logger::log_filename = "navigation.log";
bool Logger::initialized = false;

bool Logger::init() {
    std::lock_guard<std::mutex> lock(log_mutex);
    try {
        if (!initialized) {
            log_file.open(log_filename, std::ios::out | std::ios::trunc);
            if (log_file.is_open()) {
                initialized = true;
                std::time_t now = std::time(nullptr);
                log_file << "=== Log started at " 
                        << std::put_time(std::localtime(&now), "%Y-%m-%d %H:%M:%S")
                        << " ===" << std::endl;
                return true;
            }
        }
        return initialized;
    }
    catch (const std::exception& e) {
        std::cerr << "Failed to initialize logger: " << e.what() << std::endl;
        return false;
    }
}

void Logger::log(Level level, const std::string& message) {
    if (!initialized) return;

    std::lock_guard<std::mutex> lock(log_mutex);
    auto now = std::chrono::system_clock::now();
    std::time_t now_c = std::chrono::system_clock::to_time_t(now);
    
    log_file << std::put_time(std::localtime(&now_c), "%Y-%m-%d %H:%M:%S") 
             << " [" << getLevelString(level) << "] " 
             << message << std::endl;
    log_file.flush();

    // Дублируем важные сообщения в консоль
    if (level != DEBUG) {
        std::cout << "[" << getLevelString(level) << "] " << message << std::endl;
    }
}

const char* Logger::getLevelString(Level level) {
    switch (level) {
        case DEBUG:   return "DEBUG";
        case INFO:    return "INFO";
        case WARNING: return "WARNING";
        case ERROR:   return "ERROR";
        default:      return "UNKNOWN";
    }
}