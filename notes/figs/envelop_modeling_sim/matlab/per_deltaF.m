clear all; close all;

snr = 15;
num_of_dat = 127;
deltaF1 = 0;
deltaF2 = linspace(10, 20e3, 20);
per = [];
iqavg = [];
trials = 100;
for i = 1:size(deltaF2,2)
	err_num = 0;
	iqcnt = 0;
	byte_err_num = 0;
	for k = 1:trials
		[temp1, temp2, temp3] = envelop_demux(snr, num_of_dat, deltaF1, deltaF2(i));
		err_num = err_num + temp1;
		iqcnt = iqcnt + temp2;
		byte_err_num = byte_err_num + temp3;
	end
	per = [per, err_num/trials];
	iqavg = [iqavg, iqcnt/trials];
	progress = i/size(deltaF2,2);
	display(progress);
end

plot(deltaF2, per);

c = [transpose(deltaF2) transpose(per) transpose(iqavg)];
save('per.txt', '-ascii', 'c');
%{
	err_num = 0;
	iqcnt = 0;
	trials = 100;
	for k = 1:trials
		[temp1, temp2] = envelop_demux(snr, num_of_dat, 0, 0);
		err_num = err_num + temp1;
		iqcnt = iqcnt + temp2;
	end
	per = err_num/trials;
	iqavg = iqcnt/trials;

%}
