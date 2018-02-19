set term postscript enhanced eps color font "Helvetica,18" size 4in,2.5in

set output
filename = "usdr_coldboot"

set border 11 lw 2
set ytics nomirror
set xtics nomirror
set y2tics

#set lmargin 11
#set rmargin 12
#set bmargin 10 
set xlabel "Time (s)"
set ylabel "Voltage (V)"
set y2label "Current (A)"

set yrange [0:3.5]
set y2range [0:0.5]
set xrange[-0.03:0.3]
set y2tics border
set grid


#set terminal postscript eps enhanced 
#set term x11

#set key at 0.24,3 font "Arial, 20" spacing 2.5 #upper right corner 
set key out horiz
set key bottom center font "Helvetica, 16"
#set title "uSDR cold boot"

#set datafile separator ","
start_pt=10000
duration=110000
plot	filename.".txt" every ::start_pt::start_pt+duration using 1:2 title '3.3V Input' with lines ls 1 lc -1 lw 8,\
		"" every ::start_pt::start_pt+duration using 1:4 title 'Current' with lines axis x1y2 ls 2 lc 1 lw 8, \
		"" every ::start_pt::start_pt+duration using 1:3 title 'Ready' with lines ls 7 lc 4 lw 8

set output filename.".eps";
replot;

