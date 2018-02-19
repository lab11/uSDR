clear all; close all;

alpha = 0.05; % fail threshold, a portion
ratio = 1 - 2/pi*acos(sqrt(alpha));
p_single_bit = 0.2;	% single bit error probability
chip_threshold = 13;	% bigger than this value, fails

data_length = 127;	%127 bytes

%deltaF = logspace(3, 4.3, 1000);
deltaF = linspace(10, 20e3, 1000);
T = 1./deltaF;

num_of_chips = round(2e6.*(ratio.*T));	% n, ratio*T/0.5us, each chip duration is 0.5us
length_in_time = data_length * 32e-6;
num_of_envelops = length_in_time./T;	% m


per = [];
pcor = [];
for i=1:size(deltaF,2)

	p_correct = 0;
	if (((1-ratio)*T(i) - length_in_time + chip_threshold*0.5e-6/p_single_bit) > 0)
		p_correct = ((1-ratio)*T(i) - length_in_time + chip_threshold*0.5e-6)/(T(i)-length_in_time);
		total_err = 1 - p_correct;
	else
		if (num_of_chips(i)>=chip_threshold)
			for j = 0:chip_threshold-1
				p_correct = p_correct + cal_p(num_of_chips(i), j, p_single_bit);
			end
		else
			p_correct = 1;
		end
		total_err = (1 - (p_correct)^(num_of_envelops(i))); 
	end
	pcor = [pcor p_correct];
	per = [per total_err];
end
plot(deltaF, per);

temp = [transpose(deltaF) transpose(per)];
save('per_deltaf.txt', '-ascii', 'temp');

