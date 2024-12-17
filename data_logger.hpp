#ifndef DATA_LOGGER_HPP
#define DATA_LOGGER_HPP

#include <iostream>
#include <fstream>
#include <iomanip>
#include <chrono>
#include <sstream>
#include <mutex>
#include <queue>
#include <thread>
#include <atomic>
#include <ctime>
#include <vector>
#include <numeric>
#include <cmath>
#include <Eigen/Dense>
#include "logger.hpp"

// Структура для хранения позиционных данных
struct PositionData {
    double latitude;
    double longitude;
    double altitude;
    
    PositionData(double lat = 0.0, double lon = 0.0, double alt = 0.0)
        : latitude(lat), longitude(lon), altitude(alt) {}
        
    std::string toString() const {
        std::stringstream ss;
        ss << std::fixed << std::setprecision(8) 
           << latitude << "," << longitude << "," 
           << std::setprecision(2) << altitude;
        return ss.str();
    }
    
    // Вычисление расстояния до другой позиции
    double distanceTo(const PositionData& other) const {
        constexpr double EARTH_RADIUS = 6378137.0;
        constexpr double PI = 3.14159265358979323846;
        
        double dlat = (other.latitude - latitude) * PI / 180.0;
        double dlon = (other.longitude - longitude) * PI / 180.0;
        
        double a = std::sin(dlat/2) * std::sin(dlat/2) +
                  std::cos(latitude * PI / 180.0) * std::cos(other.latitude * PI / 180.0) *
                  std::sin(dlon/2) * std::sin(dlon/2);
        
        double c = 2 * std::atan2(std::sqrt(a), std::sqrt(1-a));
        double ground_distance = EARTH_RADIUS * c;
        
        double height_diff = other.altitude - altitude;
        return std::sqrt(std::pow(ground_distance, 2) + std::pow(height_diff, 2));
    }
};

// Структура для хранения логируемых данных
struct LoggedData {
    PositionData position;
    bool gnss_available;
    double hdop;
    int satellites_visible;
    double deviation;
    double distance_to_next;
    int current_waypoint;
    double current_speed_kmh;
    bool is_running;
    Eigen::Vector3d acceleration;
    Eigen::Vector3d angular_velocity;
    
    // Параметры для методов интеграции
    double ins_trust_weight;
    double gnss_trust_weight;
    double position_error;
    
    LoggedData()
        : gnss_available(false)
        , hdop(0.0)
        , satellites_visible(0)
        , deviation(0.0)
        , distance_to_next(0.0)
        , current_waypoint(0)
        , current_speed_kmh(0.0)
        , is_running(false)
        , acceleration(Eigen::Vector3d::Zero())
        , angular_velocity(Eigen::Vector3d::Zero())
        , ins_trust_weight(0.0)
        , gnss_trust_weight(0.0)
        , position_error(0.0)
    {}
};

class NavigationDataLogger {
private:
    struct DataRecord {
        std::chrono::system_clock::time_point timestamp;
        
        // Позиции от разных подсистем
        PositionData true_position;
        PositionData ins_position;
        PositionData gnss_position;
        PositionData weighted_position;
        PositionData position_based_position;
        
        // Ошибки позиционирования
        double ins_error;
        double gnss_error;
        double weighted_error;
        double position_based_error;
        
        // Веса доверия
        double ins_trust_weight;
        double gnss_trust_weight;
        
        // Навигационные данные
        double deviation;
        double distance_to_waypoint;
        int current_waypoint;
        double speed_kmh;
        
        // ГНСС данные
        bool gnss_available;
        double hdop;
        int satellites;
        
        // ИНС данные
        double acceleration_x;
        double acceleration_y;
        double acceleration_z;
        double angular_velocity_x;
        double angular_velocity_y;
        double angular_velocity_z;
        
        // Состояние системы
        bool is_running;

        std::string toCSVHeader() const;
        std::string toCSVRow() const;
    };

    std::string filename_;
    std::ofstream csv_file_;
    std::queue<DataRecord> data_queue_;
    std::mutex queue_mutex_;
    std::thread writer_thread_;
    std::atomic<bool> running_;
    std::atomic<size_t> total_records_;

    std::chrono::steady_clock::time_point last_write_time_;
    const std::chrono::milliseconds WRITE_INTERVAL{1000}; // 1 Hz
    
    void writerLoop();

public:
    NavigationDataLogger(const std::string& filename = "data/navigation_data.csv");
    ~NavigationDataLogger();
    
    void logNavigationData(
        const LoggedData& true_data,
        const LoggedData& ins_data,
        const LoggedData& gnss_data,
        const LoggedData& weighted_data,
        const LoggedData& position_based_data,
        double ins_weight,
        double gnss_weight
    );
    
    size_t getTotalRecords() const { return total_records_; }
    std::string getFilename() const { return filename_; }
};

class NavigationDataAnalyzer {
public:
    struct SystemAccuracy {
        double mean_error;
        double max_error;
        double std_dev;
        double reliability_percent;
        
        SystemAccuracy()
            : mean_error(0.0)
            , max_error(0.0)
            , std_dev(0.0)
            , reliability_percent(0.0)
        {}
    };
    
    struct AnalysisResult {
        SystemAccuracy ins_accuracy;
        SystemAccuracy gnss_accuracy;
        SystemAccuracy weighted_accuracy;
        SystemAccuracy position_based_accuracy;
        double gnss_availability_percent;
        double average_ins_weight;
        double average_gnss_weight;
        double average_deviation;
        double max_deviation;
        double average_speed;
        int total_records;
        
        AnalysisResult()
            : gnss_availability_percent(0.0)
            , average_ins_weight(0.0)
            , average_gnss_weight(0.0)
            , average_deviation(0.0)
            , max_deviation(0.0)
            , average_speed(0.0)
            , total_records(0)
        {}
    };

private:
    static std::string getCurrentTimeString();
    static SystemAccuracy calculateAccuracy(const std::vector<double>& errors);
    
public:
    static AnalysisResult analyzeData(const std::string& csv_filename);
    static void generateReport(const std::string& csv_filename, 
                             const std::string& report_filename);
    static void generateGnuplotScript(const std::string& csv_filename);
};

#endif // DATA_LOGGER_HPP