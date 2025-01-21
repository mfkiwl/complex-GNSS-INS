// logger.hpp

#ifndef LOGGER_HPP
#define LOGGER_HPP

#include <iostream>
#include <fstream>
#include <mutex>
#include <string>

class Logger {
public:
    enum Level {
        DEBUG,
        INFO,
        WARNING,
        ERROR
    };

    static bool init();
    static void log(Level level, const std::string& message);

private:
    static std::ofstream log_file;
    static std::mutex log_mutex;
    static const char* log_filename;
    static bool initialized;
    
    static const char* getLevelString(Level level);
};

#endif // LOGGER_HPP