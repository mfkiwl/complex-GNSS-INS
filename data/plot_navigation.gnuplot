# Gnuplot script for navigation data visualization
# Data file: data/navigation_data.csv
# Generated: 2024-11-08 11:56:46

# Global settings
set terminal png size 1200,800 enhanced font 'Arial,12'
set grid
set datafile separator ','
set key outside right
set style line 1 lc rgb '#0060ad' lt 1 lw 2
set style line 2 lc rgb '#dd181f' lt 1 lw 2
set style line 3 lc rgb '#00cc00' lt 1 lw 2
set style line 4 lc rgb '#ff8c00' lt 1 lw 2
set style line 5 lc rgb '#9400d3' lt 1 lw 2

# Position errors plot
set output 'data/position_errors.png'
set title 'Position Errors Over Time' font 'Arial,14'
set xlabel 'Record Number'
set ylabel 'Error (meters)'
plot 'data/navigation_data.csv' using 0:20 title 'INS' ls 1 with lines,\
     '' using 0:21 title 'GNSS' ls 2 with lines,\
     '' using 0:22 title 'Loose Integration' ls 3 with lines,\
     '' using 0:23 title 'Tight Integration' ls 4 with lines,\
     '' using 0:24 title 'Hybrid Integration' ls 5 with lines

# Trajectory plot
set output 'data/trajectory.png'
set title 'Flight Trajectory' font 'Arial,14'
set xlabel 'Longitude'
set ylabel 'Latitude'
plot 'data/navigation_data.csv' using 3:2 title 'True' ls 1 with lines,\
     '' using 6:5 title 'INS' ls 2 with lines,\
     '' using 9:8 title 'GNSS' ls 3 with lines,\
     '' using 12:11 title 'Loose' ls 4 with lines,\
     '' using 15:14 title 'Tight' ls 5 with lines

# Altitude plot
set output 'data/altitude.png'
set title 'Altitude Profile' font 'Arial,14'
set xlabel 'Record Number'
set ylabel 'Altitude (meters)'
plot 'data/navigation_data.csv' using 0:4 title 'True' ls 1 with lines,\
     '' using 0:7 title 'INS' ls 2 with lines,\
     '' using 0:10 title 'GNSS' ls 3 with lines

# Route deviation plot
set output 'data/deviation.png'
set title 'Route Deviation' font 'Arial,14'
set xlabel 'Record Number'
set ylabel 'Deviation (meters)'
plot 'data/navigation_data.csv' using 0:25 title 'Deviation' ls 1 with lines

