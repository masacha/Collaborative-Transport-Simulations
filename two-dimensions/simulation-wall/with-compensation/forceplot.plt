set xlabel "Time (s)"
set ylabel "Force (N)"
set key right top
set key inside
plot "simulation_wall.dat" using 1:8 with lines title "Bottom Robot Reaction Force", 10.0 with lines title "Force Command", "simulation_wall.dat" using 1:19 with lines title "Wall Reaction Force"
