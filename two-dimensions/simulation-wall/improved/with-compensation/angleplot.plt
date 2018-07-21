set xlabel "Time (s)"
set ylabel "Angle (rad)"
set yrange [-0.05:0.05]
set key left top
plot "simulation_wall.dat" using 1:4 with lines title "Object Yaw"
