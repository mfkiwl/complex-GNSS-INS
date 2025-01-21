// data_logger.hpp

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
struct EKFData {
    Eigen::VectorXd state;              // Вектор состояния EKF
    Eigen::MatrixXd covariance;         // Матрица ковариации
    Eigen::VectorXd innovation;         // Вектор инноваций
    double innovation_mahalanobis;      // Нормализованная инновация (NIS)
    double estimation_mahalanobis;      // Нормализованная ошибка оценивания (NEES)
    double process_noise_scale;         // Масштабный коэффициент шума процесса
    
    EKFData() 
        : state(Eigen::VectorXd::Zero(7))
        , covariance(Eigen::MatrixXd::Zero(7, 7))
        , innovation(Eigen::VectorXd::Zero(6))
        , innovation_mahalanobis(0.0)
        , estimation_mahalanobis(0.0)
        , process_noise_scale(1.0)
    {}
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
    
    EKFData ekf_data;

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
        
        // Позиционные данные от разных источников
        double true_lat, true_lon, true_alt;
        double ins_lat, ins_lon, ins_alt;
        double gnss_lat, gnss_lon, gnss_alt;
        double weighted_lat, weighted_lon, weighted_alt;
        double ekf_lat, ekf_lon, ekf_alt;
        
        // Данные EKF
        double ekf_state[7];            // [lat, lon, alt, vn, ve, vh, drift]
        double ekf_covariance_diag[7];  // Диагональные элементы ковариации
        double ekf_innovation[6];       // Инновации для измерений
        double ekf_nis;                 // Normalized Innovation Squared
        double ekf_nees;                // Normalized Estimation Error Squared
        double ekf_process_noise_scale; // Масштаб шума процесса
        
        // Параметры навигации
        double ins_weight;
        double gnss_weight;
        double route_deviation;
        double distance_to_next;
        int current_waypoint;
        double current_speed_kmh;
        double hdop;
        int satellites_visible;

         std::string toCSVRow() const;

         std::string toCSVHeader() const {
            return "Timestamp,True_Lat,True_Lon,True_Alt,INS_Lat,INS_Lon,INS_Alt,"
                   "GNSS_Lat,GNSS_Lon,GNSS_Alt,Weighted_Lat,Weighted_Lon,Weighted_Alt,"
                   "EKF_Lat,EKF_Lon,EKF_Alt,EKF_Vn,EKF_Ve,EKF_Vh,EKF_Drift,"
                   "EKF_P11,EKF_P22,EKF_P33,EKF_P44,EKF_P55,EKF_P66,EKF_P77,"
                   "EKF_Inn1,EKF_Inn2,EKF_Inn3,EKF_Inn4,EKF_Inn5,EKF_Inn6,"
                   "EKF_NIS,EKF_NEES,EKF_Q_Scale,"
                   "INS_Error,GNSS_Error,Weighted_Error,"
                   "INS_Weight,GNSS_Weight,Route_Deviation,Distance_to_WP,Current_WP,"
                   "Speed_KMH,HDOP,Satellites";
        }

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

    void fillPositionData(DataRecord& record,
                         const LoggedData& true_data,
                         const LoggedData& ins_data,
                         const LoggedData& gnss_data,
                         const LoggedData& weighted_data,
                         const LoggedData& position_based_data) 
    {
        // Заполняем истинные данные
        record.true_lat = true_data.position.latitude;
        record.true_lon = true_data.position.longitude;
        record.true_alt = true_data.position.altitude;
        
        // Данные ИНС
        record.ins_lat = ins_data.position.latitude;
        record.ins_lon = ins_data.position.longitude;
        record.ins_alt = ins_data.position.altitude;
        
        // Данные ГНСС
        record.gnss_lat = gnss_data.position.latitude;
        record.gnss_lon = gnss_data.position.longitude;
        record.gnss_alt = gnss_data.position.altitude;
        
        // Данные весовой интеграции
        record.weighted_lat = weighted_data.position.latitude;
        record.weighted_lon = weighted_data.position.longitude;
        record.weighted_alt = weighted_data.position.altitude;
        
        // Данные EKF
        record.ekf_lat = position_based_data.position.latitude;
        record.ekf_lon = position_based_data.position.longitude;
        record.ekf_alt = position_based_data.position.altitude;
    }

    void fillEKFData(DataRecord& record, const EKFData& ekf_data) {
        // Копируем состояние EKF
        for(int i = 0; i < 7; i++) {
            record.ekf_state[i] = ekf_data.state(i);
            record.ekf_covariance_diag[i] = ekf_data.covariance(i,i);
        }
        
        // Копируем инновации
        for(int i = 0; i < 6; i++) {
            record.ekf_innovation[i] = ekf_data.innovation(i);
        }
        
        // Копируем статистику EKF
        record.ekf_nis = ekf_data.innovation_mahalanobis;
        record.ekf_nees = ekf_data.estimation_mahalanobis;
        record.ekf_process_noise_scale = ekf_data.process_noise_scale;
    }

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
        double gnss_weight,
        const EKFData& ekf_data);
    
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