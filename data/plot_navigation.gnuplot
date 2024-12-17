set terminal png size 1600,1000
set grid
set datafile separator ','
# Пропускаем первую строку с заголовками
set datafile missing 'NaN'
set key outside right

# Устанавливаем формат данных
set decimal locale
set format y '%g'

set output 'data/altitude_profile.png'
set title 'Профиль высоты'
set xlabel 'Время (с)'
set ylabel 'Высота (м)'
set yrange [190:310]
# Используем every для уменьшения шума на графике
plot 'data/navigation_data.csv' every ::1 using 0:3 title 'Истинная' with lines,\
     '' every ::1 using 0:6 title 'ИНС' with lines,\
     '' every ::1 using 0:9 title 'ГНСС' with lines

set output 'data/trajectory.png'
set title 'Траектория полета'
set xlabel 'Долгота'
set ylabel 'Широта'
set size ratio -1
plot 'data/navigation_data.csv' every ::1 using 2:1 title 'Истинная' with lines,\
     '' every ::1 using 5:4 title 'ИНС' with lines,\
     '' every ::1 using 8:7 title 'ГНСС' with lines,\
     '' every ::1 using 11:10 title 'Весовая интеграция' with lines,\
     '' every ::1 using 14:13 title 'Поз./скор. интеграция' with lines

set output 'data/gnss_quality.png'
set title 'Качество сигнала ГНСС'
set xlabel 'Время (с)'
set ylabel 'HDOP'
set y2label 'Количество спутников'
set ytics nomirror
set y2tics
set yrange [0:*]
set y2range [0:15]
plot 'data/navigation_data.csv' every ::1 using 0:28 title 'HDOP' with lines axis x1y1,\
     '' every ::1 using 0:29 title 'Спутники' with lines axis x1y2

set output 'data/ekf_convergence.png'
set title 'Сходимость оценок EKF'
set xlabel 'Время (с)'
set ylabel 'Ошибка (м)'
# Вычисляем ошибку как расстояние между истинным и оцененным положением
plot 'data/navigation_data.csv' every ::1 using 0:(sqrt(($2-$14)*($2-$14) + ($1-$13)*($1-$13))) title 'Ошибка EKF' with lines

set output 'data/ekf_innovation_sequence.png'
set title 'Инновационная последовательность EKF'
set xlabel 'Время (с)'
set ylabel 'Невязка (м)'
# Вычисляем невязки как разность между измерениями ГНСС и оценками EKF
plot 'data/navigation_data.csv' every ::1 using 0:($8-$14) title 'Невязка по широте' with lines,\
     '' every ::1 using 0:($7-$13) title 'Невязка по долготе' with lines
