clear all; close all;

snr = 10;
num_of_dat = linspace(7, 127, 11);
deltaF1 = -10e3;
deltaF2 = 5e3;
per = [];
iqavg = []
trials = 100;
for i = 1:size(num_of_dat,2)
	err_num = 0;
	iqcnt = 0;
	for k = 1:trials
		[temp1, temp2] = envelop_demux(snr, num_of_dat(i), deltaF1, deltaF2);
		err_num = err_num + temp1;
		iqcnt = iqcnt + temp2;
	end
	per = [per, err_num/trials];
	iqavg = [iqavg, iqcnt/trials];
end

plot(num_of_dat, per);

c = [transpose(num_of_dat) transpose(per) transpose(iqavg)];
save('per.txt', '-ascii', 'c');
