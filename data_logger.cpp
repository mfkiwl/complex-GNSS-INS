#include "data_logger.hpp"
#include <cstdlib>
#include <cerrno>
#include <cstring>

std::string NavigationDataLogger::DataRecord::toCSVRow() const {
    auto time_t = std::chrono::system_clock::to_time_t(timestamp);
    std::stringstream ss;
    ss << std::put_time(std::localtime(&time_t), "%Y-%m-%d %H:%M:%S") << ",";
    
    // Позиции
    ss << true_position.toString() << ","
       << ins_position.toString() << ","
       << gnss_position.toString() << ","
       << loose_position.toString() << ","
       << tight_position.toString() << ","
       << hybrid_position.toString() << ",";
    
    // Ошибки
    ss << std::fixed << std::setprecision(3)
       << ins_error << "," << gnss_error << ","
       << loose_error << "," << tight_error << ","
       << hybrid_error << ",";
    
    // Навигационные данные
    ss << std::fixed << std::setprecision(2)
       << deviation << "," << distance_to_waypoint << ","
       << current_waypoint << "," << speed_kmh << ",";
    
    // ГНСС данные
    ss << (gnss_available ? "1" : "0") << ","
       << std::setprecision(3) << hdop << ","
       << satellites << ",";
    
    // ИНС данные
    ss << std::setprecision(6)
       << acceleration_x << "," << acceleration_y << "," << acceleration_z << ","
       << angular_velocity_x << "," << angular_velocity_y << "," << angular_velocity_z << ",";
    
    // Состояние системы
    ss << (is_running ? "1" : "0") << ","
       << (tight_coupling_active ? "1" : "0");
    
    return ss.str();
}

void NavigationDataLogger::writerLoop() {
    last_write_time_ = std::chrono::steady_clock::now();
    
    while (running_ || !data_queue_.empty()) {
        auto current_time = std::chrono::steady_clock::now();
        
        // Записываем данные только раз в секунду
        if (current_time - last_write_time_ >= WRITE_INTERVAL) {
            std::vector<DataRecord> batch;
            {
                std::lock_guard<std::mutex> lock(queue_mutex_);
                while (!data_queue_.empty()) {
                    batch.push_back(data_queue_.front());
                    data_queue_.pop();
                }
            }
            
            if (!batch.empty()) {
                for (const auto& record : batch) {
                    csv_file_ << record.toCSVRow() << std::endl;
                }
                csv_file_.flush();
            }
            
            last_write_time_ = current_time;
        }
        
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

NavigationDataLogger::NavigationDataLogger(const std::string& filename)
    : filename_(filename)
    , running_(false)
    , total_records_(0)
    , last_write_time_(std::chrono::steady_clock::now())
{
    // Создаем директорию для данных
    std::string data_dir = "data";
    if (system(("mkdir -p " + data_dir).c_str()) != 0) {
        Logger::log(Logger::ERROR, "Failed to create data directory: " + std::string(strerror(errno)));
        throw std::runtime_error("Failed to create data directory");
    }

    Logger::log(Logger::INFO, "Created data directory: " + data_dir);

    // Открываем файл для записи
    csv_file_.open(filename_, std::ios::out | std::ios::trunc);
    if (!csv_file_.is_open()) {
        Logger::log(Logger::ERROR, "Failed to open CSV file: " + filename_ + 
                   " (" + std::string(strerror(errno)) + ")");
        throw std::runtime_error("Failed to open CSV file: " + filename_);
    }
    
    Logger::log(Logger::INFO, "Opened CSV file for writing: " + filename_);
    
    // Записываем заголовок
    DataRecord dummy;
    csv_file_ << dummy.toCSVHeader() << std::endl;
    
    if (!csv_file_.good()) {
        Logger::log(Logger::ERROR, "Failed to write CSV header");
        throw std::runtime_error("Failed to write CSV header");
    }
    
    running_ = true;
    writer_thread_ = std::thread(&NavigationDataLogger::writerLoop, this);
    
    Logger::log(Logger::INFO, "Data logger initialized successfully");
}

NavigationDataLogger::~NavigationDataLogger() {
    Logger::log(Logger::INFO, "Shutting down data logger...");
    running_ = false;
    
    if (writer_thread_.joinable()) {
        writer_thread_.join();
    }
    
    if (csv_file_.is_open()) {
        csv_file_.close();
    }
    
    Logger::log(Logger::INFO, "Data logger shut down. Total records written: " + std::to_string(total_records_));
}

void NavigationDataLogger::logNavigationData(
    const LoggedData& true_data,
    const LoggedData& ins_data,
    const LoggedData& gnss_data,
    const LoggedData& loose_data,
    const LoggedData& tight_data,
    const LoggedData& hybrid_data,
    bool tight_coupling_active)
{
    DataRecord record;
    record.timestamp = std::chrono::system_clock::now();
    
    // Копируем позиции
    record.true_position = true_data.position;
    record.ins_position = ins_data.position;
    record.gnss_position = gnss_data.position;
    record.loose_position = loose_data.position;
    record.tight_position = tight_data.position;
    record.hybrid_position = hybrid_data.position;
    
    // Вычисляем ошибки
    record.ins_error = record.true_position.distanceTo(record.ins_position);
    record.gnss_error = record.true_position.distanceTo(record.gnss_position);
    record.loose_error = record.true_position.distanceTo(record.loose_position);
    record.tight_error = record.true_position.distanceTo(record.tight_position);
    record.hybrid_error = record.true_position.distanceTo(record.hybrid_position);
    
    // Копируем навигационные данные
    record.deviation = true_data.deviation;
    record.distance_to_waypoint = true_data.distance_to_next;
    record.current_waypoint = true_data.current_waypoint;
    record.speed_kmh = true_data.current_speed_kmh;
    record.gnss_available = true_data.gnss_available;
    record.hdop = true_data.hdop;
    record.satellites = true_data.satellites_visible;
    
    // Копируем данные ИНС
    record.acceleration_x = true_data.acceleration.x();
    record.acceleration_y = true_data.acceleration.y();
    record.acceleration_z = true_data.acceleration.z();
    record.angular_velocity_x = true_data.angular_velocity.x();
    record.angular_velocity_y = true_data.angular_velocity.y();
    record.angular_velocity_z = true_data.angular_velocity.z();
    
    // Состояние системы
    record.is_running = true_data.is_running;
    record.tight_coupling_active = tight_coupling_active;
    
    // Добавляем запись в очередь
    {
        std::lock_guard<std::mutex> lock(queue_mutex_);
        data_queue_.push(record);
    }
    
    total_records_++;
}

void NavigationDataAnalyzer::generateGnuplotScript(const std::string& csv_filename) {
    try {
        // Убедимся, что директория существует
        std::string data_dir = "data";
        if (system(("mkdir -p " + data_dir).c_str()) != 0) {
            throw std::runtime_error("Failed to create data directory");
        }

        // Проверяем наличие данных
        std::ifstream check_data(csv_filename);
        if (!check_data.good()) {
            throw std::runtime_error("Data file not found or empty: " + csv_filename);
        }

        Logger::log(Logger::INFO, "Found data file: " + csv_filename);

        std::string script_file = data_dir + "/plot_navigation.gnuplot";
        Logger::log(Logger::INFO, "Creating gnuplot script: " + script_file);

        std::ofstream script(script_file);
        if (!script.is_open()) {
            throw std::runtime_error("Failed to create gnuplot script file: " + script_file);
        }

        // Записываем скрипт
        std::time_t current_time = std::time(nullptr);
script << "# Gnuplot script for navigation data visualization\n"
       << "# Data file: " << csv_filename << "\n"
       << "# Generated: " << std::put_time(std::localtime(&current_time), "%Y-%m-%d %H:%M:%S") << "\n\n"
               
               << "# Global settings\n"
               << "set terminal png size 1200,800 enhanced font 'Arial,12'\n"
               << "set grid\n"
               << "set datafile separator ','\n"
               << "set key outside right\n"
               << "set style line 1 lc rgb '#0060ad' lt 1 lw 2\n"
               << "set style line 2 lc rgb '#dd181f' lt 1 lw 2\n"
               << "set style line 3 lc rgb '#00cc00' lt 1 lw 2\n"
               << "set style line 4 lc rgb '#ff8c00' lt 1 lw 2\n"
               << "set style line 5 lc rgb '#9400d3' lt 1 lw 2\n\n"

               // График ошибок позиционирования
               << "# Position errors plot\n"
               << "set output '" << data_dir << "/position_errors.png'\n"
               << "set title 'Position Errors Over Time' font 'Arial,14'\n"
               << "set xlabel 'Record Number'\n"
               << "set ylabel 'Error (meters)'\n"
               << "plot '" << csv_filename << "' using 0:20 title 'INS' ls 1 with lines,\\\n"
               << "     '' using 0:21 title 'GNSS' ls 2 with lines,\\\n"
               << "     '' using 0:22 title 'Loose Integration' ls 3 with lines,\\\n"
               << "     '' using 0:23 title 'Tight Integration' ls 4 with lines,\\\n"
               << "     '' using 0:24 title 'Hybrid Integration' ls 5 with lines\n\n"

               // График траектории
               << "# Trajectory plot\n"
               << "set output '" << data_dir << "/trajectory.png'\n"
               << "set title 'Flight Trajectory' font 'Arial,14'\n"
               << "set xlabel 'Longitude'\n"
               << "set ylabel 'Latitude'\n"
               << "plot '" << csv_filename << "' using 3:2 title 'True' ls 1 with lines,\\\n"
               << "     '' using 6:5 title 'INS' ls 2 with lines,\\\n"
               << "     '' using 9:8 title 'GNSS' ls 3 with lines,\\\n"
               << "     '' using 12:11 title 'Loose' ls 4 with lines,\\\n"
               << "     '' using 15:14 title 'Tight' ls 5 with lines\n\n"

               // График высоты
               << "# Altitude plot\n"
               << "set output '" << data_dir << "/altitude.png'\n"
               << "set title 'Altitude Profile' font 'Arial,14'\n"
               << "set xlabel 'Record Number'\n"
               << "set ylabel 'Altitude (meters)'\n"
               << "plot '" << csv_filename << "' using 0:4 title 'True' ls 1 with lines,\\\n"
               << "     '' using 0:7 title 'INS' ls 2 with lines,\\\n"
               << "     '' using 0:10 title 'GNSS' ls 3 with lines\n\n"

               // График отклонения от маршрута
               << "# Route deviation plot\n"
               << "set output '" << data_dir << "/deviation.png'\n"
               << "set title 'Route Deviation' font 'Arial,14'\n"
               << "set xlabel 'Record Number'\n"
               << "set ylabel 'Deviation (meters)'\n"
               << "plot '" << csv_filename << "' using 0:25 title 'Deviation' ls 1 with lines\n\n";

        script.close();

        if (!script) {
            throw std::runtime_error("Failed to write gnuplot script");
        }

        // Устанавливаем права на выполнение
        if (system(("chmod +x " + script_file).c_str()) != 0) {
            Logger::log(Logger::WARNING, "Failed to set execute permissions on script file");
        }

        Logger::log(Logger::INFO, "Gnuplot script created successfully");
    }
    catch (const std::exception& e) {
        Logger::log(Logger::ERROR, "Error generating gnuplot script: " + std::string(e.what()));
        throw;
    }
}

NavigationDataAnalyzer::AnalysisResult 
NavigationDataAnalyzer::analyzeData(const std::string& csv_filename) {
    std::ifstream csv_file(csv_filename);
    if (!csv_file.is_open()) {
        throw std::runtime_error("Failed to open CSV file: " + csv_filename);
    }
    
    // Пропускаем заголовок
    std::string header;
    std::getline(csv_file, header);
    
    AnalysisResult result = {};
    std::vector<double> ins_errors, gnss_errors, loose_errors, tight_errors, hybrid_errors;
    std::vector<double> deviations, speeds;
    int gnss_available_count = 0;
    int tight_coupling_count = 0;
    
    std::string line;
    while (std::getline(csv_file, line)) {
        std::stringstream ss(line);
        std::string field;
        std::vector<std::string> fields;
        
        while (std::getline(ss, field, ',')) {
            fields.push_back(field);
        }
        
        if (fields.size() >= 40) {
            ins_errors.push_back(std::stod(fields[19]));
            gnss_errors.push_back(std::stod(fields[20]));
            loose_errors.push_back(std::stod(fields[21]));
            tight_errors.push_back(std::stod(fields[22]));
            hybrid_errors.push_back(std::stod(fields[23]));
            
            deviations.push_back(std::stod(fields[24]));
            speeds.push_back(std::stod(fields[27]));
            
            if (fields[29] == "1") gnss_available_count++;
            if (fields[39] == "1") tight_coupling_count++;
            
            result.total_records++;
        }
}
    
    auto calculate_accuracy = [](const std::vector<double>& errors) -> SystemAccuracy {
        SystemAccuracy acc;
        
        if (errors.empty()) {
            Logger::log(Logger::WARNING, "No data for accuracy calculation");
            return acc;
        }
        
        // Среднее значение ошибки
        acc.mean_error = std::accumulate(errors.begin(), errors.end(), 0.0) / errors.size();
        
        // Максимальная ошибка
        acc.max_error = *std::max_element(errors.begin(), errors.end());
        
        // Стандартное отклонение
        double sq_sum = std::inner_product(errors.begin(), errors.end(), errors.begin(), 0.0);
        acc.std_dev = std::sqrt(sq_sum / errors.size() - acc.mean_error * acc.mean_error);
        
        // Надежность (% времени с ошибкой < 10м)
        int reliable_count = std::count_if(errors.begin(), errors.end(), 
                                         [](double err) { return err < 10.0; });
        acc.reliability_percent = (double)reliable_count / errors.size() * 100.0;
        
        return acc;
    };
    
    // Заполняем результаты анализа
    result.ins_accuracy = calculate_accuracy(ins_errors);
    result.gnss_accuracy = calculate_accuracy(gnss_errors);
    result.loose_accuracy = calculate_accuracy(loose_errors);
    result.tight_accuracy = calculate_accuracy(tight_errors);
    result.hybrid_accuracy = calculate_accuracy(hybrid_errors);
    
    if (result.total_records > 0) {
        // Вычисляем остальные показатели
        result.gnss_availability_percent = (double)gnss_available_count / result.total_records * 100.0;
        result.tight_coupling_usage_percent = (double)tight_coupling_count / result.total_records * 100.0;
        result.average_deviation = std::accumulate(deviations.begin(), deviations.end(), 0.0) / 
                                 deviations.size();
        result.max_deviation = *std::max_element(deviations.begin(), deviations.end());
        result.average_speed = std::accumulate(speeds.begin(), speeds.end(), 0.0) / speeds.size();
    }
    
    Logger::log(Logger::INFO, "Data analysis completed. Processed " + 
               std::to_string(result.total_records) + " records");
    
    return result;
}

void NavigationDataAnalyzer::generateReport(const std::string& csv_filename, 
                                          const std::string& report_filename) {
    try {
        auto result = analyzeData(csv_filename);
        
        std::ofstream report_file(report_filename);
        if (!report_file.is_open()) {
            throw std::runtime_error("Failed to create report file: " + report_filename);
        }
        
        std::time_t current_time = std::time(nullptr);
report_file << "=== Navigation System Performance Analysis ===\n"
           << "Generated: " << std::put_time(std::localtime(&current_time), 
                                            "%Y-%m-%d %H:%M:%S") << "\n\n"
                   << "Total records analyzed: " << result.total_records << "\n\n";
        
        auto print_system_accuracy = [&report_file](const std::string& name, 
                                                  const SystemAccuracy& acc) {
            report_file << name << " Performance:\n"
                       << std::fixed << std::setprecision(2)
                       << "  Mean error: " << acc.mean_error << " m\n"
                       << "  Maximum error: " << acc.max_error << " m\n"
                       << "  Standard deviation: " << acc.std_dev << " m\n"
                       << "  Reliability (<10m): " << acc.reliability_percent << "%\n\n";
        };
        
        print_system_accuracy("INS", result.ins_accuracy);
        print_system_accuracy("GNSS", result.gnss_accuracy);
        print_system_accuracy("Loose Integration", result.loose_accuracy);
        print_system_accuracy("Tight Integration", result.tight_accuracy);
        print_system_accuracy("Hybrid Integration", result.hybrid_accuracy);
        
        report_file << "System Statistics:\n"
                   << std::fixed << std::setprecision(1)
                   << "  GNSS Availability: " << result.gnss_availability_percent << "%\n"
                   << "  Tight Coupling Usage: " << result.tight_coupling_usage_percent << "%\n"
                   << "  Average Route Deviation: " << result.average_deviation << " m\n"
                   << "  Maximum Route Deviation: " << result.max_deviation << " m\n"
                   << "  Average Speed: " << result.average_speed << " km/h\n\n";
        
        report_file << "Comparative Analysis:\n";
        std::vector<std::pair<std::string, double>> mean_errors = {
            {"INS", result.ins_accuracy.mean_error},
            {"GNSS", result.gnss_accuracy.mean_error},
            {"Loose Integration", result.loose_accuracy.mean_error},
            {"Tight Integration", result.tight_accuracy.mean_error},
            {"Hybrid Integration", result.hybrid_accuracy.mean_error}
        };
        
        auto best_system = std::min_element(mean_errors.begin(), mean_errors.end(),
            [](const auto& a, const auto& b) { return a.second < b.second; });
            
        report_file << "  Best performing system: " << best_system->first 
                   << " (mean error: " << best_system->second << " m)\n\n";
                   
        // Рекомендации
        report_file << "System Recommendations:\n";
        if (result.gnss_availability_percent < 90.0) {
            report_file << "  - Consider improving GNSS reception or antenna placement\n";
        }
        if (result.hybrid_accuracy.mean_error > result.tight_accuracy.mean_error) {
            report_file << "  - Review hybrid integration algorithm parameters\n";
        }
        if (result.ins_accuracy.std_dev > 10.0) {
            report_file << "  - INS calibration may be required\n";
        }
        
        report_file << "\nNote: This report was generated from the data file: " 
                   << csv_filename << std::endl;

        Logger::log(Logger::INFO, "Analysis report generated: " + report_filename);
    }
    catch (const std::exception& e) {
        Logger::log(Logger::ERROR, "Error generating analysis report: " + std::string(e.what()));
        throw;
    }
}
