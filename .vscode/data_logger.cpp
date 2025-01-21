#include "data_logger.hpp"
#include <cstdlib>
#include <cerrno>
#include <cstring>

std::string NavigationDataLogger::DataRecord::toCSVHeader() const {
    return "Timestamp,"
           "True_Lat,True_Lon,True_Alt,"
           "INS_Lat,INS_Lon,INS_Alt,"
           "GNSS_Lat,GNSS_Lon,GNSS_Alt,"
           "Weighted_Lat,Weighted_Lon,Weighted_Alt,"
           "PosBased_Lat,PosBased_Lon,PosBased_Alt,"
           "INS_Error,GNSS_Error,Weighted_Error,PosBased_Error,"
           "INS_Weight,GNSS_Weight,"
           "Route_Deviation,Distance_to_WP,Current_WP,Speed_KMH,"
           "GNSS_Available,HDOP,Satellites,"
           "Accel_X,Accel_Y,Accel_Z,"
           "AngVel_X,AngVel_Y,AngVel_Z,"
           "Is_Running";
}

std::string NavigationDataLogger::DataRecord::toCSVRow() const {
    auto time_t = std::chrono::system_clock::to_time_t(timestamp);
    std::stringstream ss;
    ss << std::put_time(std::localtime(&time_t), "%Y-%m-%d %H:%M:%S") << ",";
    
    ss << true_position.toString() << ","
       << ins_position.toString() << ","
       << gnss_position.toString() << ","
       << weighted_position.toString() << ","
       << position_based_position.toString() << ",";
    
    ss << std::fixed << std::setprecision(3)
       << ins_error << "," << gnss_error << ","
       << weighted_error << "," << position_based_error << ",";
    
    ss << std::fixed << std::setprecision(3)
       << ins_trust_weight << "," << gnss_trust_weight << ",";
    
    ss << std::fixed << std::setprecision(2)
       << deviation << "," << distance_to_waypoint << ","
       << current_waypoint << "," << speed_kmh << ",";
    
    ss << (gnss_available ? "1" : "0") << ","
       << std::setprecision(3) << hdop << ","
       << satellites << ",";
    
    ss << std::setprecision(6)
       << acceleration_x << "," << acceleration_y << "," << acceleration_z << ","
       << angular_velocity_x << "," << angular_velocity_y << "," << angular_velocity_z << ",";
    
    ss << (is_running ? "1" : "0");
    
    return ss.str();
}

void NavigationDataLogger::writerLoop() {
    last_write_time_ = std::chrono::steady_clock::now();
    
    while (running_ || !data_queue_.empty()) {
        auto current_time = std::chrono::steady_clock::now();
        
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
    std::string data_dir = "data";
    if (system(("mkdir -p " + data_dir).c_str()) != 0) {
        Logger::log(Logger::ERROR, "Не удалось создать директорию данных: " + 
                   std::string(strerror(errno)));
        throw std::runtime_error("Не удалось создать директорию данных");
    }

    csv_file_.open(filename_, std::ios::out | std::ios::trunc);
    if (!csv_file_.is_open()) {
        Logger::log(Logger::ERROR, "Не удалось открыть CSV файл: " + filename_);
        throw std::runtime_error("Не удалось открыть CSV файл: " + filename_);
    }
    
    DataRecord dummy;
    csv_file_ << dummy.toCSVHeader() << std::endl;
    
    if (!csv_file_.good()) {
        Logger::log(Logger::ERROR, "Не удалось записать заголовок CSV");
        throw std::runtime_error("Не удалось записать заголовок CSV");
    }
    
    running_ = true;
    writer_thread_ = std::thread(&NavigationDataLogger::writerLoop, this);
    
    Logger::log(Logger::INFO, "Логгер данных успешно инициализирован");
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
    const LoggedData& weighted_data,
    const LoggedData& position_based_data,
    double ins_weight,
    double gnss_weight)
{
    DataRecord record;
    record.timestamp = std::chrono::system_clock::now();
    
    record.true_position = true_data.position;
    record.ins_position = ins_data.position;
    record.gnss_position = gnss_data.position;
    record.weighted_position = weighted_data.position;
    record.position_based_position = position_based_data.position;
    
    record.ins_error = ins_data.position_error;
    record.gnss_error = record.true_position.distanceTo(record.gnss_position);
    record.weighted_error = record.true_position.distanceTo(record.weighted_position);
    record.position_based_error = record.true_position.distanceTo(record.position_based_position);
    
    record.ins_trust_weight = ins_weight;
    record.gnss_trust_weight = gnss_weight;
    
    record.deviation = true_data.deviation;
    record.distance_to_waypoint = true_data.distance_to_next;
    record.current_waypoint = true_data.current_waypoint;
    record.speed_kmh = true_data.current_speed_kmh;
    
    record.gnss_available = true_data.gnss_available;
    record.hdop = true_data.hdop;
    record.satellites = true_data.satellites_visible;
    
    record.acceleration_x = true_data.acceleration.x();
    record.acceleration_y = true_data.acceleration.y();
    record.acceleration_z = true_data.acceleration.z();
    record.angular_velocity_x = true_data.angular_velocity.x();
    record.angular_velocity_y = true_data.angular_velocity.y();
    record.angular_velocity_z = true_data.angular_velocity.z();
    
    record.is_running = true_data.is_running;
    
    {
        std::lock_guard<std::mutex> lock(queue_mutex_);
        data_queue_.push(record);
    }
    
    total_records_++;
}

NavigationDataAnalyzer::SystemAccuracy 
NavigationDataAnalyzer::calculateAccuracy(const std::vector<double>& errors) {
    SystemAccuracy acc;
    if (errors.empty()) {
        return acc;
    }

    // Среднее значение ошибки
    acc.mean_error = std::accumulate(errors.begin(), errors.end(), 0.0) / errors.size();

    // Максимальная ошибка
    acc.max_error = *std::max_element(errors.begin(), errors.end());

    // Стандартное отклонение
    double sq_sum = std::inner_product(errors.begin(), errors.end(), errors.begin(), 0.0);
    acc.std_dev = std::sqrt(sq_sum / errors.size() - acc.mean_error * acc.mean_error);

    // Надежность (процент значений с ошибкой < 10м)
    int reliable_count = std::count_if(errors.begin(), errors.end(), 
                                     [](double err) { return err < 10.0; });
    acc.reliability_percent = (double)reliable_count / errors.size() * 100.0;

    return acc;
}

NavigationDataAnalyzer::AnalysisResult 
NavigationDataAnalyzer::analyzeData(const std::string& csv_filename) {
    AnalysisResult result;
    std::ifstream csv_file(csv_filename);
    if (!csv_file.is_open()) {
        throw std::runtime_error("Не удалось открыть CSV файл: " + csv_filename);
    }

    std::string header, line;
    std::getline(csv_file, header);

    std::vector<double> ins_errors;
    std::vector<double> gnss_errors;
    std::vector<double> weighted_errors;
    std::vector<double> pos_based_errors;
    std::vector<double> ins_weights;
    std::vector<double> gnss_weights;
    std::vector<double> deviations;
    std::vector<double> speeds;

    while (std::getline(csv_file, line)) {
        if (line.empty()) continue;

        std::stringstream ss(line);
        std::string field;
        std::vector<std::string> fields;

        while (std::getline(ss, field, ',')) {
            fields.push_back(field);
        }

        if (fields.size() >= 35) {
            ins_errors.push_back(std::stod(fields[16]));
            gnss_errors.push_back(std::stod(fields[17]));
            weighted_errors.push_back(std::stod(fields[18]));
            pos_based_errors.push_back(std::stod(fields[19]));

            ins_weights.push_back(std::stod(fields[20]));
            gnss_weights.push_back(std::stod(fields[21]));

            deviations.push_back(std::stod(fields[22]));
            speeds.push_back(std::stod(fields[25]));

            if (fields[26] == "1") {
                result.gnss_availability_percent++;
            }

            result.total_records++;
        }
    }

    if (result.total_records > 0) {
        result.ins_accuracy = calculateAccuracy(ins_errors);
        result.gnss_accuracy = calculateAccuracy(gnss_errors);
        result.weighted_accuracy = calculateAccuracy(weighted_errors);
        result.position_based_accuracy = calculateAccuracy(pos_based_errors);

        result.average_ins_weight = std::accumulate(ins_weights.begin(), ins_weights.end(), 0.0) / 
                                  ins_weights.size();
        result.average_gnss_weight = std::accumulate(gnss_weights.begin(), gnss_weights.end(), 0.0) / 
                                   gnss_weights.size();
        result.average_deviation = std::accumulate(deviations.begin(), deviations.end(), 0.0) / 
                                 deviations.size();
        result.max_deviation = *std::max_element(deviations.begin(), deviations.end());
        result.average_speed = std::accumulate(speeds.begin(), speeds.end(), 0.0) / speeds.size();
        
        result.gnss_availability_percent = (result.gnss_availability_percent / result.total_records) * 100.0;
    }

    return result;
}

void NavigationDataAnalyzer::generateReport(const std::string& csv_filename, 
                                          const std::string& report_filename) {
    auto result = analyzeData(csv_filename);
    
    std::ifstream csv_file(csv_filename);
    if (!csv_file.is_open()) {
        throw std::runtime_error("Не удалось открыть CSV файл: " + csv_filename);
    }

    std::string line, header;
    std::getline(csv_file, header);

    std::vector<double> position_errors;
    int total_samples = 0;
    int within_3sigma = 0;
    double sum_squared_error = 0.0;

    while (std::getline(csv_file, line)) {
        std::stringstream ss(line);
        std::vector<double> values;
        std::string value;
        
        while (std::getline(ss, value, ',')) {
            values.push_back(std::stod(value));
        }

        double true_lat = values[1];    // True_Lat
        double true_lon = values[2];    // True_Lon
        double ekf_lat = values[13];    // PosBased_Lat
        double ekf_lon = values[14];    // PosBased_Lon
        
        double position_error = std::sqrt(
            std::pow(true_lat - ekf_lat, 2) + 
            std::pow(true_lon - ekf_lon, 2)
        );

        position_errors.push_back(position_error);
        sum_squared_error += position_error * position_error;
        total_samples++;

        double current_rmse = std::sqrt(sum_squared_error / total_samples);
        if (position_error <= 3 * current_rmse) {
            within_3sigma++;
        }
    }

    double rmse = std::sqrt(sum_squared_error / total_samples);
    double within_3sigma_percent = (double)within_3sigma / total_samples * 100.0;

    std::ofstream report(report_filename);
    if (!report.is_open()) {
        throw std::runtime_error("Не удалось создать файл отчета: " + report_filename);
    }

    report << "=== Анализ методов интеграции навигационной системы ===\n"
           << "Проанализировано записей: " << result.total_records << "\n\n"
           
           << "1. Весовая интеграция:\n"
           << "   Средняя ошибка: " << result.weighted_accuracy.mean_error << " м\n"
           << "   Максимальная ошибка: " << result.weighted_accuracy.max_error << " м\n"
           << "   Надежность (<10м): " << result.weighted_accuracy.reliability_percent << "%\n"
           << "   Стандартное отклонение: " << result.weighted_accuracy.std_dev << " м\n"
           << "   Средние веса - ИНС: " << result.average_ins_weight 
           << ", ГНСС: " << result.average_gnss_weight << "\n\n"
           
           << "2. Позиционно-скоростная интеграция (EKF):\n"
           << "   Среднеквадратическая ошибка (RMSE): " << std::fixed << std::setprecision(3) 
           << rmse << " м\n"
           << "   Максимальная ошибка: " << result.position_based_accuracy.max_error << " м\n"
           << "   Стандартное отклонение: " << result.position_based_accuracy.std_dev << " м\n"
           << "   Доля оценок в пределах 3σ: " << std::fixed << std::setprecision(1) 
           << within_3sigma_percent << "%\n"
           << "   Надежность (<10м): " 
           << result.position_based_accuracy.reliability_percent << "%\n\n"

           << "Общая статистика:\n"
           << "- Доступность ГНСС: " << result.gnss_availability_percent << "%\n"
           << "- Среднее отклонение от маршрута: " << result.average_deviation << " м\n"
           << "- Средняя скорость: " << result.average_speed << " км/ч\n\n"

           << "Дополнительные характеристики EKF:\n"
           << "- СКО оценки положения: " << std::fixed << std::setprecision(3) << rmse << " м\n"
           << "- Процент оценок в пределах 3σ: " << std::fixed << std::setprecision(1) 
           << within_3sigma_percent << "%\n"
           << "- Максимальное отклонение: " << std::fixed << std::setprecision(3) 
           << *std::max_element(position_errors.begin(), position_errors.end()) << " м\n";

    report.close();
    csv_file.close();
}

void NavigationDataAnalyzer::generateGnuplotScript(const std::string& csv_filename) {
    try {
        std::string data_dir = "data";
        if (system(("mkdir -p " + data_dir).c_str()) != 0) {
            throw std::runtime_error("Не удалось создать директорию данных");
        }

        std::string script_file = data_dir + "/plot_navigation.gnuplot";
        std::ofstream script(script_file);
        if (!script.is_open()) {
            throw std::runtime_error("Не удалось создать файл скрипта: " + script_file);
        }

        // Базовые настройки
        script << "set terminal png size 1600,1000\n"
               << "set grid\n"
               << "set datafile separator ','\n"
               << "# Пропускаем первую строку с заголовками\n"
               << "set datafile missing 'NaN'\n"
               << "set key outside right\n\n"
               << "# Устанавливаем формат данных\n"
               << "set decimal locale\n"
               << "set format y '%g'\n\n";

        // График профиля высоты
        // Колонки: True_Alt = 3, INS_Alt = 6, GNSS_Alt = 9
        script << "set output '" << data_dir << "/altitude_profile.png'\n"
               << "set title 'Профиль высоты'\n"
               << "set xlabel 'Время (с)'\n"
               << "set ylabel 'Высота (м)'\n"
               << "set yrange [190:310]\n"
               << "# Используем every для уменьшения шума на графике\n"
               << "plot '" << csv_filename << "' every ::1 using 0:3 title 'Истинная' with lines,\\\n"
               << "     '' every ::1 using 0:6 title 'ИНС' with lines,\\\n"
               << "     '' every ::1 using 0:9 title 'ГНСС' with lines\n\n";

        // График траектории
        // Колонки: True_Lat/Lon = 1,2; INS_Lat/Lon = 4,5; GNSS_Lat/Lon = 7,8; 
        // Weighted_Lat/Lon = 10,11; PosBased_Lat/Lon = 13,14
        script << "set output '" << data_dir << "/trajectory.png'\n"
               << "set title 'Траектория полета'\n"
               << "set xlabel 'Долгота'\n"
               << "set ylabel 'Широта'\n"
               << "set size ratio -1\n"
               << "plot '" << csv_filename << "' every ::1 using 2:1 title 'Истинная' with lines,\\\n"
               << "     '' every ::1 using 5:4 title 'ИНС' with lines,\\\n"
               << "     '' every ::1 using 8:7 title 'ГНСС' with lines,\\\n"
               << "     '' every ::1 using 11:10 title 'Весовая интеграция' with lines,\\\n"
               << "     '' every ::1 using 14:13 title 'Поз./скор. интеграция' with lines\n\n";

        // График HDOP и спутников
        // Колонки: HDOP = 28, Satellites = 29
        script << "set output '" << data_dir << "/gnss_quality.png'\n"
               << "set title 'Качество сигнала ГНСС'\n"
               << "set xlabel 'Время (с)'\n"
               << "set ylabel 'HDOP'\n"
               << "set y2label 'Количество спутников'\n"
               << "set ytics nomirror\n"
               << "set y2tics\n"
               << "set yrange [0:*]\n"
               << "set y2range [0:15]\n"
               << "plot '" << csv_filename << "' every ::1 using 0:28 title 'HDOP' with lines axis x1y1,\\\n"
               << "     '' every ::1 using 0:29 title 'Спутники' with lines axis x1y2\n\n";

        // График сходимости EKF
        script << "set output '" << data_dir << "/ekf_convergence.png'\n"
               << "set title 'Сходимость оценок EKF'\n"
               << "set xlabel 'Время (с)'\n"
               << "set ylabel 'Ошибка (м)'\n"
               << "# Вычисляем ошибку как расстояние между истинным и оцененным положением\n"
               << "plot '" << csv_filename << "' every ::1 using 0:(sqrt(($2-$14)*($2-$14) + ($1-$13)*($1-$13))) title 'Ошибка EKF' with lines\n\n";

        // График инновационной последовательности
        script << "set output '" << data_dir << "/ekf_innovation_sequence.png'\n"
               << "set title 'Инновационная последовательность EKF'\n"
               << "set xlabel 'Время (с)'\n"
               << "set ylabel 'Невязка (м)'\n"
               << "# Вычисляем невязки как разность между измерениями ГНСС и оценками EKF\n"
               << "plot '" << csv_filename << "' every ::1 using 0:($8-$14) title 'Невязка по широте' with lines,\\\n"
               << "     '' every ::1 using 0:($7-$13) title 'Невязка по долготе' with lines\n";

         script << "# График NEES\n"
           << "set output '" << data_dir << "/ekf_nees.png'\n"
           << "set title 'Normalized Estimation Error Squared (NEES)'\n"
           << "set xlabel 'Время (с)'\n"
           << "set ylabel 'NEES'\n"
           << "set yrange [0:*]\n"
           << "# Добавляем линию 95% доверительного интервала\n"
           << "set arrow from 0,7.81 to GPVAL_X_MAX,7.81 nohead lt 2 lc rgb 'red'\n"
           << "plot '" << csv_filename << "' using 0:($NEES_COLUMN) title 'NEES' with lines\n\n";

    script << "# График NIS\n"
           << "set output '" << data_dir << "/ekf_nis.png'\n"
           << "set title 'Normalized Innovation Squared (NIS)'\n"
           << "set xlabel 'Время (с)'\n"
           << "set ylabel 'NIS'\n"
           << "set yrange [0:*]\n"
           << "# Добавляем линию 95% доверительного интервала\n"
           << "set arrow from 0,5.99 to GPVAL_X_MAX,5.99 nohead lt 2 lc rgb 'red'\n"
           << "plot '" << csv_filename << "' using 0:($NIS_COLUMN) title 'NIS' with lines\n\n";

    script << "# График автокорреляции инноваций\n"
           << "set output '" << data_dir << "/ekf_innovation_autocorr.png'\n"
           << "set title 'Автокорреляция инновационной последовательности'\n"
           << "set xlabel 'Лаг'\n"
           << "set ylabel 'Коэффициент автокорреляции'\n"
           << "set yrange [-1:1]\n"
           << "# Добавляем доверительные интервалы\n"
           << "set arrow from 0,0.2 to GPVAL_X_MAX,0.2 nohead lt 2 lc rgb 'red'\n"
           << "set arrow from 0,-0.2 to GPVAL_X_MAX,-0.2 nohead lt 2 lc rgb 'red'\n"
           << "plot '" << csv_filename << "' using 0:($AUTOCORR_COLUMN) title 'Автокорреляция' with lines\n\n";

    script << "# График ошибок оценки позиции\n"
           << "set output '" << data_dir << "/ekf_position_errors.png'\n"
           << "set title 'Ошибки оценки позиции EKF'\n"
           << "set xlabel 'Время (с)'\n"
           << "set ylabel 'Ошибка (м)'\n"
           << "set yrange [0:*]\n"
           << "plot '" << csv_filename << "' using 0:($LAT_ERROR_COLUMN) title 'Ошибка по широте' with lines,\\\n"
           << "     '' using 0:($LON_ERROR_COLUMN) title 'Ошибка по долготе' with lines\n\n";

    script << "# График 3σ границ\n"
           << "set output '" << data_dir << "/ekf_3sigma_bounds.png'\n"
           << "set title 'Ошибки оценки с 3σ границами'\n"
           << "set xlabel 'Время (с)'\n"
           << "set ylabel 'Ошибка (м)'\n"
           << "set y2label '3σ границы'\n"
           << "plot '" << csv_filename << "' using 0:($ERROR_COLUMN) title 'Ошибка оценки' with lines,\\\n"
           << "     '' using 0:($SIGMA_BOUNDS_COLUMN) title '3σ границы' with lines lt 2\n";

        script.close();
        
        if (!script) {
            throw std::runtime_error("Ошибка при записи скрипта gnuplot");
        }

        Logger::log(Logger::INFO, "Скрипт gnuplot успешно создан");
    }
    catch (const std::exception& e) {
        Logger::log(Logger::ERROR, "Ошибка при генерации скрипта gnuplot: " + std::string(e.what()));
        throw;
    }
}