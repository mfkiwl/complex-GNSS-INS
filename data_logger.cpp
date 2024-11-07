#include "data_logger.hpp"

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
    while (running_ || !data_queue_.empty()) {
        std::vector<DataRecord> batch;
        {
            std::lock_guard<std::mutex> lock(queue_mutex_);
            while (!data_queue_.empty() && batch.size() < 100) {
                batch.push_back(data_queue_.front());
                data_queue_.pop();
            }
        }
        
        for (const auto& record : batch) {
            csv_file_ << record.toCSVRow() << std::endl;
        }
        
        if (batch.empty()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }
}

NavigationDataLogger::NavigationDataLogger(const std::string& filename)
    : filename_(filename)
    , running_(false)
    , total_records_(0)
{
    csv_file_.open(filename_, std::ios::out | std::ios::trunc);
    if (!csv_file_.is_open()) {
        throw std::runtime_error("Failed to open CSV file: " + filename_);
    }
    
    DataRecord dummy;
    csv_file_ << dummy.toCSVHeader() << std::endl;
    
    running_ = true;
    writer_thread_ = std::thread(&NavigationDataLogger::writerLoop, this);
}

NavigationDataLogger::~NavigationDataLogger() {
    running_ = false;
    if (writer_thread_.joinable()) {
        writer_thread_.join();
    }
    if (csv_file_.is_open()) {
        csv_file_.close();
    }
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
            // Индексы соответствуют структуре CSV файла
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
    
    // Вычисляем статистику для каждой системы
    auto calculate_accuracy = [](const std::vector<double>& errors) -> SystemAccuracy {
        SystemAccuracy acc;
        
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
    
    // Вычисляем остальные показатели
    result.gnss_availability_percent = (double)gnss_available_count / result.total_records * 100.0;
    result.tight_coupling_usage_percent = (double)tight_coupling_count / result.total_records * 100.0;
    result.average_deviation = std::accumulate(deviations.begin(), deviations.end(), 0.0) / 
                             deviations.size();
    result.max_deviation = *std::max_element(deviations.begin(), deviations.end());
    result.average_speed = std::accumulate(speeds.begin(), speeds.end(), 0.0) / speeds.size();
    
    return result;
}

void NavigationDataAnalyzer::generateReport(const std::string& csv_filename, 
                                          const std::string& report_filename) {
    auto result = analyzeData(csv_filename);
    
    std::ofstream report_file(report_filename);
    if (!report_file.is_open()) {
        throw std::runtime_error("Failed to create report file: " + report_filename);
    }
    
    report_file << "=== Navigation System Performance Analysis ===\n\n";
    report_file << "Total records analyzed: " << result.total_records << "\n\n";
    
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
}

void NavigationDataAnalyzer::generateGnuplotScript(const std::string& csv_filename) {
    std::ofstream script_file("plot_navigation.gnuplot");
    if (!script_file.is_open()) {
        throw std::runtime_error("Failed to create Gnuplot script file");
    }
    
    script_file << "set terminal png size 1200,800 enhanced font 'Arial,10'\n"
               << "set grid\n"
               << "set datafile separator ','\n\n"
               
               // Общие настройки для всех графиков
               << "set style line 1 lc rgb '#1f77b4' lw 2\n"  // синий
               << "set style line 2 lc rgb '#ff7f0e' lw 2\n"  // оранжевый
               << "set style line 3 lc rgb '#2ca02c' lw 2\n"  // зеленый
               << "set style line 4 lc rgb '#d62728' lw 2\n"  // красный
               << "set style line 5 lc rgb '#9467bd' lw 2\n"  // фиолетовый
               << "set style line 6 lc rgb '#8c564b' lw 2\n"  // коричневый
               << "\n"
               
               // График ошибок позиционирования
               << "set output 'data/position_errors.png'\n"
               << "set title 'Position Errors Over Time'\n"
               << "set xlabel 'Record Number'\n"
               << "set ylabel 'Error (meters)'\n"
               << "set key outside\n"
               << "set key right top\n"
               << "plot '" << csv_filename << "' using 0:20 title 'INS' with lines ls 1, \\\n"
               << "     '' using 0:21 title 'GNSS' with lines ls 2, \\\n"
               << "     '' using 0:22 title 'Loose Integration' with lines ls 3, \\\n"
               << "     '' using 0:23 title 'Tight Integration' with lines ls 4, \\\n"
               << "     '' using 0:24 title 'Hybrid Integration' with lines ls 5\n\n"
               
               // График траектории
               << "set output 'data/trajectory.png'\n"
               << "set title 'Flight Trajectory'\n"
               << "set xlabel 'Longitude'\n"
               << "set ylabel 'Latitude'\n"
               << "plot '" << csv_filename << "' using 3:2 title 'True' with lines ls 1, \\\n"
               << "     '' using 6:5 title 'INS' with lines ls 2, \\\n"
               << "     '' using 9:8 title 'GNSS' with lines ls 3, \\\n"
               << "     '' using 12:11 title 'Loose' with lines ls 4, \\\n"
               << "     '' using 15:14 title 'Tight' with lines ls 5, \\\n"
               << "     '' using 18:17 title 'Hybrid' with lines ls 6\n\n"
               
               // График высоты
               << "set output 'data/altitude.png'\n"
               << "set title 'Altitude Profile'\n"
               << "set xlabel 'Record Number'\n"
               << "set ylabel 'Altitude (meters)'\n"
               << "plot '" << csv_filename << "' using 0:4 title 'True' with lines ls 1, \\\n"
               << "     '' using 0:7 title 'INS' with lines ls 2, \\\n"
               << "     '' using 0:10 title 'GNSS' with lines ls 3\n\n"
               
               // График отклонения от маршрута
               << "set output 'data/deviation.png'\n"
               << "set title 'Route Deviation'\n"
               << "set xlabel 'Record Number'\n"
               << "set ylabel 'Deviation (meters)'\n"
               << "plot '" << csv_filename << "' using 0:25 title 'Deviation' with lines ls 1\n\n"
               
               // График доступности ГНСС
               << "set output 'data/gnss_quality.png'\n"
               << "set title 'GNSS Quality Parameters'\n"
               << "set ylabel 'Value'\n"
               << "set y2label 'Satellites'\n"
               << "set ytics nomirror\n"
               << "set y2tics\n"
               << "plot '" << csv_filename << "' using 0:30 title 'HDOP' with lines ls 1, \\\n"
               << "     '' using 0:31 title 'Satellites' with lines ls 2 axes x1y2\n";
}