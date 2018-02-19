clear all; close all;

data = csvread('usrp_e100_run.csv', 17);
time = data(:,1);
dc_input = data(:,4);
current = 0.552*data(:,2);

clear data;
window_size = 10;

current = medfilt1(current, window_size);
dc_input = medfilt1(dc_input, window_size);

data = [time, dc_input, current];

save ('usrp_e100_run.txt', 'data', '-ascii');
