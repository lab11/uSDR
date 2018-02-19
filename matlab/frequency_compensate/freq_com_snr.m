clear all; close all;

error_array = [];
num_of_trials = 100;
freq_resolution = 300;	% step size = 300HZ
data_length = 64;		% 64 bytes
for snr = 10:50
    error_cal = [];
	for i = 1:num_of_trials
		[out] = freq_compensate(snr, data_length, freq_resolution);
		error_cal = [error_cal out];
	end
	error_array = [error_array sqrt(mean(error_cal.^2))];
end

snr = 10:50;
data = [transpose(snr)  transpose(error_array)];
%semilogy(data(:,1), data(:,2));
save('freq_snr.txt', 'data', '-ascii');
grid on;

