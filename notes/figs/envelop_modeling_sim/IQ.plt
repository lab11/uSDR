set term postscript enhanced eps color font "Helvetica,24" 

folder = "matlab"
filename = "IQ"

set border 3 lw 2
set ytics nomirror
set xtics nomirror

set xlabel "time (us)"
set ylabel "amplitude"

set grid

set key out horiz spacing 5 
set key bottom center

set output filename.".eps"

set size 2.5,1

set multiplot
set size 1.2,0.9
set xrange [1e2:3e2]
set yrange [-200:200]
set origin 0.05,0.05
plot	folder."/".filename.".txt" using ($1*1e6):2 title "In-phase" with lines ls 1 lc 1 lw 8

set origin 1.3,0.05
plot	folder."/".filename.".txt" using ($1*1e6):3 title "Quad-phase" with lines ls 1 lc 4 lw 8
unset multiplot

#replot
