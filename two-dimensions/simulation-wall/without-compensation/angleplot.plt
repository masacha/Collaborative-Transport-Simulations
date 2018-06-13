set xlabel "Time (s)"
set ylabel "Angle (rad)"
set key right top
set key inside
plot "simulation_wall.dat" using 1:7 with lines title "Object Yaw"
