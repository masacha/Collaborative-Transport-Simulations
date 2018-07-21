set xlabel "Time (s)"
set ylabel "Force (N)"
set yrange [0:3.0]
set xrange [0:10]
set key right top
set key inside
plot "simulation_wall.dat" using 1:8 with lines title "Bottom1 Robot Reaction Force", "simulation_wall.dat" using 1:12 with lines title "Bottom2 Robot Reaction Force", "simulation_wall.dat" using 1:($8+$12) with lines title "Total Force from Robots", 5 with lines title "Force Command"
