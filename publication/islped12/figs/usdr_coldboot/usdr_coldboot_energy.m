clear all; close all;

data = csvread('usdr_coldboot.csv', 17);
time = data(:,1);
dc_input = data(:,2);
system_ready = data(:,4);
current = data(:,6)/192*20;
sampling_period = time(2) - time(1);
supply_vol = 4.8;

clear data;
window_size = 10;

current = medfilt1(current, window_size);
dc_input = medfilt1(dc_input, window_size);
system_ready = medfilt1(system_ready, window_size);

data = [time, dc_input, system_ready, current];
save ('usdr_coldboot.txt', 'data', '-ascii');

%{
time_start = 0;
time_stop = 0.259;	% observe from graph

[diff, start_idx] = min(abs(time - time_start));
[diff, stop_idx] = min(abs(time - time_stop));

sum_i_t = sampling_period * sum(current(start_idx:stop_idx));
cold_boot_energy = supply_vol * sum_i_t; % unit: Joule
%}
