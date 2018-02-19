set term postscript enhanced eps color font "Helvetica,18" size 4in,2.5in

set output
folder = "matlab"
filename = "per_deltaf_sim"

set output filename.".eps"
set border 3 lw 2
set ytics nomirror
set xtics nomirror

#set lmargin 11
#set bmargin 9 
set xlabel "{/Symbol D}f KHz"
set ylabel "Perket Error Rate %"

#set yrange [80:95]
set grid

#set term x11
#set terminal postscript eps enhanced 

set key out horiz spacing 5 
set key bottom center
#set key at 135, 99 font "Arial, 20" spacing 2.5
plot	folder."/".filename.".txt" using ($1/1000):($2*100) title "PER" with lines ls 1 lc 1 lw 8

#replot

