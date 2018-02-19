set term postscript enhanced eps color font "Helvetica,18" size 4in,2.5in

folder = "matlab"
filename = "freq_compensate"

set output filename.".eps"
set border 3 lw 2
set ytics nomirror
set xtics nomirror

set xlabel "SNR (dB)"
set ylabel "RMS Frequency error (KHz)"

set grid

#set term x11
#set terminal postscript eps enhanced 

set key out horiz spacing 5 
set key bottom center
#set key at 135, 99 font "Arial, 20" spacing 2.5
plot	folder."/".filename.".txt" using 1:($2/1e3) title "127 Bytes" with lines ls 1 lc 1 lw 8,\
		"" using 1:($3*10) title "64 Bytes" with lines ls 1 lc 4 lw 8

#replot

