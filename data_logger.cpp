// data_logger.cpp

#include "data_logger.hpp"
#include <cstdlib>
#include <cerrno>
#include <cstring>

std::string NavigationDataLogger::DataRecord::toCSVRow() const {
     // Используем stringstream для форматированного вывода чисел с нужной точностью
    std::stringstream ss;
    
    // Форматируем временную метку в читаемый вид
    auto time_t = std::chrono::system_clock::to_time_t(timestamp);
    ss << std::put_time(std::localtime(&time_t), "%Y-%m-%d %H:%M:%S") << ",";
    
    // Записываем истинные координаты с высокой точностью (9 знаков после запятой)
    ss << std::fixed << std::setprecision(9)
       << true_lat << "," << true_lon << "," 
       << std::setprecision(3) << true_alt << ",";  // Высота с меньшей точностью
    
    // Координаты от ИНС
    ss << std::setprecision(9)
       << ins_lat << "," << ins_lon << ","
       << std::setprecision(3) << ins_alt << ",";
    
    // Координаты от ГНСС
    ss << std::setprecision(9)
       << gnss_lat << "," << gnss_lon << ","
       << std::setprecision(3) << gnss_alt << ",";
    
    // Координаты после весовой интеграции
    ss << std::setprecision(9)
       << weighted_lat << "," << weighted_lon << ","
       << std::setprecision(3) << weighted_alt << ",";
    
    // Координаты от EKF
    ss << std::setprecision(9)
       << ekf_lat << "," << ekf_lon << ","
       << std::setprecision(3) << ekf_alt << ",";
    
    // Полный вектор состояния EKF
    ss << std::setprecision(6);
    for(int i = 0; i < 7; i++) {
        ss << ekf_state[i] << ",";
    }
    
    // Диагональные элементы ковариационной матрицы EKF
    ss << std::scientific << std::setprecision(6);  // Экспоненциальный формат для малых чисел
    for(int i = 0; i < 7; i++) {
        ss << ekf_covariance_diag[i] << ",";
    }
    
    // Вектор инноваций EKF
    ss << std::fixed << std::setprecision(6);
    for(int i = 0; i < 6; i++) {
        ss << ekf_innovation[i] << ",";
    }
    
    // Статистика работы EKF
    ss << ekf_nis << ","
       << ekf_nees << ","
       << ekf_process_noise_scale << ",";
    
    // Ошибки различных подсистем
    ss << std::setprecision(3)
       << ins_weight << ","
       << gnss_weight << ","
       << route_deviation << ","
       << distance_to_next << ","
       << current_waypoint << ","
       << std::setprecision(1) << current_speed_kmh << ",";
    
    // Параметры качества ГНСС
    ss << std::setprecision(2) << hdop << ","
       << satellites_visible;
    
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
    double gnss_weight,
    const EKFData& ekf_data)
{
    try {
        DataRecord record;
        record.timestamp = std::chrono::system_clock::now();
        
        // Заполняем все поля записи
        fillPositionData(record, true_data, ins_data, gnss_data, 
                        weighted_data, position_based_data);
        fillEKFData(record, ekf_data);
        
        // Заполняем навигационные параметры
        record.ins_weight = ins_weight;
        record.gnss_weight = gnss_weight;
        record.route_deviation = true_data.deviation;
        record.distance_to_next = true_data.distance_to_next;
        record.current_waypoint = true_data.current_waypoint;
        record.current_speed_kmh = true_data.current_speed_kmh;
        record.hdop = gnss_data.hdop;
        record.satellites_visible = gnss_data.satellites_visible;
        
        // Добавляем запись в очередь
        {
            std::lock_guard<std::mutex> lock(queue_mutex_);
            data_queue_.push(record);
            total_records_++;
        }
    }
    catch (const std::exception& e) {
        Logger::log(Logger::ERROR, "Error logging navigation data: " + std::string(e.what()));
    }
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
            throw std::runtime_error("Failed to create data directory");
        }

        std::string script_file = data_dir + "/plot_navigation.gnuplot";
        std::ofstream script(script_file);
        if (!script.is_open()) {
            throw std::runtime_error("Failed to create gnuplot script file");
        }

        // Базовые настройки Gnuplot
        script << "# Базовые настройки для всех графиков\n"
               << "set terminal png size 1600,1000\n"
               << "set grid\n"
               << "set datafile separator ','\n"
               << "set datafile missing 'NaN'\n"
               << "set key outside right\n"
               << "set style line 1 lc rgb '#0060ad' lw 2\n"
               << "set style line 2 lc rgb '#dd181f' lw 2\n"
               << "set style line 3 lc rgb '#00cc00' lw 2\n"
               << "set style line 4 lc rgb '#ff9900' lw 2\n\n";

        // График траектории в горизонтальной плоскости
        script << "# Траектория движения БПЛА\n"
               << "set output '" << data_dir << "/trajectory.png'\n"
               << "set title 'Траектория полета'\n"
               << "set xlabel 'Долгота'\n"
               << "set ylabel 'Широта'\n"
               << "set size ratio -1\n"
               << "plot '" << csv_filename << "' every ::1 using 3:2 title 'Истинная' with lines ls 1,\\\n"
               << "     '' every ::1 using 6:5 title 'ИНС' with lines ls 2,\\\n"
               << "     '' every ::1 using 9:8 title 'ГНСС' with lines ls 3,\\\n"
               << "     '' every ::1 using 12:11 title 'Весовая интеграция' with lines ls 4,\\\n"
               << "     '' every ::1 using 15:14 title 'EKF' with lines ls 5\n\n";

        // График изменения высоты
        script << "# Профиль высоты\n"
               << "set output '" << data_dir << "/altitude_profile.png'\n"
               << "set title 'Изменение высоты'\n"
               << "set xlabel 'Время (с)'\n"
               << "set ylabel 'Высота (м)'\n"
               << "plot '" << csv_filename << "' every ::1 using ($0/10):4 title 'Истинная' with lines ls 1,\\\n"
               << "     '' every ::1 using ($0/10):7 title 'ИНС' with lines ls 2,\\\n"
               << "     '' every ::1 using ($0/10):10 title 'ГНСС' with lines ls 3,\\\n"
               << "     '' every ::1 using ($0/10):16 title 'EKF' with lines ls 5\n\n";

        // График ошибок позиционирования
        script << "# Ошибки определения координат\n"
               << "set output '" << data_dir << "/position_errors.png'\n"
               << "set title 'Ошибки определения координат'\n"
               << "set xlabel 'Время (с)'\n"
               << "set ylabel 'Ошибка (м)'\n"
               << "plot '" << csv_filename << "' every ::1 using ($0/10):37 title 'ИНС' with lines ls 2,\\\n"
               << "     '' every ::1 using ($0/10):38 title 'ГНСС' with lines ls 3,\\\n"
               << "     '' every ::1 using ($0/10):39 title 'Весовая интеграция' with lines ls 4,\\\n"
               << "     '' every ::1 using ($0/10):40 title 'EKF' with lines ls 5\n\n";

        // График весовых коэффициентов
        script << "# Весовые коэффициенты интеграции\n"
               << "set output '" << data_dir << "/weights.png'\n"
               << "set title 'Весовые коэффициенты'\n"
               << "set xlabel 'Время (с)'\n"
               << "set ylabel 'Вес'\n"
               << "set yrange [0:1]\n"
               << "plot '" << csv_filename << "' every ::1 using ($0/10):41 title 'Вес ИНС' with lines ls 1,\\\n"
               << "     '' every ::1 using ($0/10):42 title 'Вес ГНСС' with lines ls 2\n\n";

        // График статистики ГНСС
        script << "# Качество сигнала ГНСС\n"
               << "set output '" << data_dir << "/gnss_quality.png'\n"
               << "set multiplot layout 2,1\n"
               << "set title 'Параметры качества ГНСС'\n"
               << "set xlabel 'Время (с)'\n"
               << "set ylabel 'HDOP'\n"
               << "plot '" << csv_filename << "' every ::1 using ($0/10):47 title 'HDOP' with lines ls 1\n"
               << "set ylabel 'Количество спутников'\n"
               << "plot '" << csv_filename << "' every ::1 using ($0/10):48 title 'Видимые спутники' with lines ls 2\n"
               << "unset multiplot\n\n";

        // Графики для анализа работы EKF
        script << "# Анализ работы EKF\n"
               << "set output '" << data_dir << "/ekf_innovations.png'\n"
               << "set title 'Инновации EKF с границами 3σ'\n"
               << "set xlabel 'Время (с)'\n"
               << "set ylabel 'Инновация'\n"
               << "plot '" << csv_filename << "' every ::1 using ($0/10):28 title 'Инновация позиции' with lines ls 1,\\\n"
               << "     '' every ::1 using ($0/10):($28+3*sqrt($21)) title '+3σ' with lines ls 2,\\\n"
               << "     '' every ::1 using ($0/10):($28-3*sqrt($21)) title '-3σ' with lines ls 2\n\n";

        // График NIS/NEES
        script << "# Консистентность оценок EKF\n"
               << "set output '" << data_dir << "/ekf_consistency.png'\n"
               << "set multiplot layout 2,1\n"
               << "set title 'Показатели согласованности EKF'\n"
               << "set xlabel 'Время (с)'\n"
               << "set ylabel 'NIS'\n"
               << "plot '" << csv_filename << "' every ::1 using ($0/10):34 title 'NIS' with lines ls 1,\\\n"
               << "     12.59 title '95% χ²' with lines ls 2\n"
               << "set ylabel 'NEES'\n"
               << "plot '" << csv_filename << "' every ::1 using ($0/10):35 title 'NEES' with lines ls 1,\\\n"
               << "     7.81 title '95% χ²' with lines ls 2\n"
               << "unset multiplot\n\n";

        // График адаптивного масштаба шума процесса
        script << "# Адаптивная настройка EKF\n"
               << "set output '" << data_dir << "/ekf_adaptive.png'\n"
               << "set title 'Масштаб шума процесса'\n"
               << "set xlabel 'Время (с)'\n"
               << "set ylabel 'Масштабный коэффициент'\n"
               << "plot '" << csv_filename << "' every ::1 using ($0/10):36 title 'Масштаб Q' with lines ls 1\n";

        script.close();
        Logger::log(Logger::INFO, "Gnuplot script generated successfully");
    }
    catch (const std::exception& e) {
        Logger::log(Logger::ERROR, "Error generating gnuplot script: " + std::string(e.what()));
        throw;
    }
}