
set term postscript enhanced eps color font "Helvetica,18" size 4in,2.5in
set output
filename = "usrp_e100_run"

set border 11 lw 2
set ytics nomirror
set xtics nomirror
set y2tics nomirror

set xlabel "Time (s)"
set ylabel "Voltage (V)"
set y2label "Current (A)"

set yrange [2:3.5]
set y2range [0:2.5]
#set xrange [-3:5]
set y2tics border
set grid front


#set term x11

set key out horiz width 3
set key bottom center
#set title "USRP E100"

set datafile separator ","
start_pt=1000
duration=100000
plot	filename.".csv" every ::start_pt using 1:4 title 'firmware loaded' with lines ls 2 lc 4 lw 4, \
		"" 				every ::start_pt using 1:($2*0.552) title 'system current' with lines axis x1y2 ls 1 lc 1 lw 4

set label "Idle" at first -4.7, first 2.9 font "Helvetica,11"
set label "Loading\nFirmware" at first -3.8, first 2.9 font "Helvetica,11"
set label "Creating Control Object" at first -2.2, first 2.9 font "Helvetica,11"
set label "Transmitting Data" at first 1.5, first 2.9 font "Helvetica,11"
set style fill transparent solid 0.5 noborder
set style rect back fillcolor lt -1 fs solid 0.1 noborder
set obj rect from -3.95, graph 0 to -2.35, graph 1
set obj rect from 1.15, graph 0 to 4.3, graph 1
set output filename.".eps";
replot;

