function [out] = freq_offset_gen(snr, num_of_dat, deltaF)

[rx_I, rx_Q, gen_seq] = oqpsk_gen(snr, num_of_dat);

f_sample = 16e6;
T_sample = 1/f_sample;
t = 0:T_sample:(size(rx_I,2)-1)*T_sample;

%deltaF = 50e3;
phi = round(rand()*360);
phi = phi/180*pi;

rx_I_deltaF = int8(rx_I.*cos(2*pi*(deltaF).*t + phi) + rx_Q.*sin(2*pi*(deltaF).*t + phi));
rx_Q_deltaF = int8(rx_Q.*cos(2*pi*(deltaF).*t + phi) - rx_I.*sin(2*pi*(deltaF).*t + phi));

out_dat = [transpose(rx_I_deltaF), transpose(rx_Q_deltaF)];
dlmwrite('IQ_deltaF.txt', out_dat, 'delimiter', '\t');

%{
ang_ori = atan2(rx_Q, rx_I).*(180/pi);
ang_deltaF = atan2(rx_Q_deltaF, rx_I_deltaF).*(180/pi);
ang_ori_diff = zeros(1, size(ang_ori, 2)-1);
ang_deltaF_diff = zeros(1, size(ang_ori, 2)-1);

for i = 1:size(rx_I,2)-1
	ang_ori_diff(i) = (ang_ori(i+1) - ang_ori(i));
	ang_deltaF_diff(i) = (ang_deltaF(i+1) - ang_deltaF(i));
	if (ang_ori_diff(i) < -180)
		ang_ori_diff(i) = ang_ori_diff(i) + 360;
	elseif (ang_ori_diff(i) > 180)
		ang_ori_diff(i) = ang_ori_diff(i) - 360;
	end

	if (ang_deltaF_diff(i) < -180)
		ang_deltaF_diff(i) = ang_deltaF_diff(i) + 360;
	elseif (ang_deltaF_diff(i) > 180)
		ang_deltaF_diff(i) = ang_deltaF_diff(i) - 360;
	end

end

plot(ang_ori_diff);
hold;
plot(ang_deltaF_diff, 'r');
grid on;
%}
