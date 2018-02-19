clear all; close all;

num_of_sec = 31;
Fs = 44100;
num_of_samples = num_of_sec * Fs;

y = wavread('hotel_california.wav', num_of_samples, 'native');

x = int32(y);
x = x + 32768;
%{
x = uint16(x);

dlmwrite('audio_crop.txt', x, 'delimiter', '\t');
%}

x = uint32(x);
out = bitshift(x(:,1), 16) + x(:,2);
dlmwrite('audio_crop2.txt', out, 'precision', '%10d');
