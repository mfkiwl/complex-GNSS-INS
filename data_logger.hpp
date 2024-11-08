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
        PositionData loose_position;
        PositionData tight_position;
        PositionData hybrid_position;
        
        // Ошибки позиционирования
        double ins_error;
        double gnss_error;
        double loose_error;
        double tight_error;
        double hybrid_error;
        
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
        bool tight_coupling_active;
        
        std::string toCSVHeader() const {
            return "Timestamp,"
                   "True_Lat,True_Lon,True_Alt,"
                   "INS_Lat,INS_Lon,INS_Alt,"
                   "GNSS_Lat,GNSS_Lon,GNSS_Alt,"
                   "Loose_Lat,Loose_Lon,Loose_Alt,"
                   "Tight_Lat,Tight_Lon,Tight_Alt,"
                   "Hybrid_Lat,Hybrid_Lon,Hybrid_Alt,"
                   "INS_Error,GNSS_Error,Loose_Error,Tight_Error,Hybrid_Error,"
                   "Route_Deviation,Distance_to_WP,Current_WP,Speed_KMH,"
                   "GNSS_Available,HDOP,Satellites,"
                   "Accel_X,Accel_Y,Accel_Z,"
                   "AngVel_X,AngVel_Y,AngVel_Z,"
                   "Is_Running,Tight_Coupling_Active";
        }
        
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
    
    void logNavigationData(const LoggedData& true_data,
                          const LoggedData& ins_data,
                          const LoggedData& gnss_data,
                          const LoggedData& loose_data,
                          const LoggedData& tight_data,
                          const LoggedData& hybrid_data,
                          bool tight_coupling_active);
    
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
    };
    
    struct AnalysisResult {
        SystemAccuracy ins_accuracy;
        SystemAccuracy gnss_accuracy;
        SystemAccuracy loose_accuracy;
        SystemAccuracy tight_accuracy;
        SystemAccuracy hybrid_accuracy;
        double gnss_availability_percent;
        double tight_coupling_usage_percent;
        double average_deviation;
        double max_deviation;
        double average_speed;
        int total_records;
    };
    
    static AnalysisResult analyzeData(const std::string& csv_filename);
    static void generateReport(const std::string& csv_filename, 
                             const std::string& report_filename);
    static void generateGnuplotScript(const std::string& csv_filename);
};

#endif // DATA_LOGGER_HPP