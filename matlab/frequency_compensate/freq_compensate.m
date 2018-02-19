function [out] = freq_compensate(snr, num_of_dat, freq_resolution)

demapping= [1, 1, 0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 1, 1, 1, 1, 0, 1, 0, 1, 1, 1, 0, 0, 1, 1, 0, 1, 1, 0, 0; 
			1, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 1, 1, 1, 1, 0, 1, 0, 1, 1, 1, 0, 0, 1, 1, 0;
			1, 1, 0, 1, 1, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 1, 1, 1, 1, 0, 1, 0, 1, 1, 1, 0;
			1, 1, 0, 0, 1, 1, 0, 1, 1, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 1, 1, 1, 1, 0, 1, 0;
			0, 1, 0, 1, 1, 1, 0, 0, 1, 1, 0, 1, 1, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 1, 1, 1;
			1, 1, 1, 1, 0, 1, 0, 1, 1, 1, 0, 0, 1, 1, 0, 1, 1, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0, 1, 1, 1;
			1, 1, 1, 0, 1, 1, 1, 1, 0, 1, 0, 1, 1, 1, 0, 0, 1, 1, 0, 1, 1, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0;
			0, 0, 0, 0, 1, 1, 1, 0, 1, 1, 1, 1, 0, 1, 0, 1, 1, 1, 0, 0, 1, 1, 0, 1, 1, 0, 0, 1, 1, 1, 0;
			0, 0, 1, 1, 1, 1, 1, 1, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 1, 0, 0, 0, 1, 1, 0, 0, 1, 0, 0, 1, 1;
			0, 1, 1, 0, 0, 0, 1, 1, 1, 1, 1, 1, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 1, 0, 0, 0, 1, 1, 0, 0, 1;
			0, 0, 1, 0, 0, 1, 1, 0, 0, 0, 1, 1, 1, 1, 1, 1, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 1, 0, 0, 0, 1;
			0, 0, 1, 1, 0, 0, 1, 0, 0, 1, 1, 0, 0, 0, 1, 1, 1, 1, 1, 1, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 1;
			1, 0, 1, 0, 0, 0, 1, 1, 0, 0, 1, 0, 0, 1, 1, 0, 0, 0, 1, 1, 1, 1, 1, 1, 0, 0, 0, 1, 0, 0, 0;
			0, 0, 0, 0, 1, 0, 1, 0, 0, 0, 1, 1, 0, 0, 1, 0, 0, 1, 1, 0, 0, 0, 1, 1, 1, 1, 1, 1, 0, 0, 0;
			0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 1, 0, 0, 0, 1, 1, 0, 0, 1, 0, 0, 1, 1, 0, 0, 0, 1, 1, 1, 1, 1;
			1, 1, 1, 1, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 1, 0, 0, 0, 1, 1, 0, 0, 1, 0, 0, 1, 1, 0, 0, 0, 1];

[rx_I, rx_Q, gen_seq] = oqpsk_gen(snr, num_of_dat);

f_sample = 16e6;
T_sample = 1/f_sample;
t = 0:T_sample:(size(rx_I,2)-1)*T_sample;

%{
subplot(2,1,1);
plot(t, rx_I);
xlabel('time');
grid on;

subplot(2,1,2);
plot(t, rx_Q);
xlabel('time');
grid on;
%}



%deltaF = -30e3 + 60e3.*rand(1, 1); 	% generate an -100k to 100k offset
deltaF = 50e3;
phi = round(rand()*360);
phi = phi/180*pi;
%deltaF = 0;
%phi = 0;


val(1) = 0;
seq_arr = zeros(8,31);
val_integ(1:7) = zeros(1, 7);
decoded_bit(1:7) = zeros(1, 7);
sync = 0;
seq_cnt = 0;
freq_comp = 0;
diff_arr = [];
state = 0;
packet_length = 127;
packet_counter = 0;
rx_length = 0;
for i = 1:size(rx_I, 2)
	rx_I_deltaF(i) = rx_I(i)*cos(2*pi*(deltaF + freq_comp)*i*T_sample+ phi) + rx_Q(i)*sin(2*pi*(deltaF + freq_comp)*i*T_sample+ phi);
	rx_Q_deltaF(i) = rx_Q(i)*cos(2*pi*(deltaF + freq_comp)*i*T_sample+ phi) - rx_I(i)*sin(2*pi*(deltaF + freq_comp)*i*T_sample+ phi);

	% demux
	if (i>1)
		val(i) = rx_Q_deltaF(i)*rx_I_deltaF(i-1) - rx_Q_deltaF(i-1)*rx_I_deltaF(i);
	end

	% match filter
	if (i>7)
		val_integ(i) = sum(val(i-7:i));
		if (val_integ(i)>0)
			decoded_bit(i) = 1;
		else
			decoded_bit(i) = 0;
		end
	end

	% Assign to 8 rows, each row is down sampled to 2 MHz
	row_idx = mod(i,8) + 1;
	entire_row = seq_arr(row_idx,:);
	seq_arr(row_idx,:) = [entire_row(2:31) decoded_bit(i)];

	% finding 0x00
	if (sync==0)
		if (mod(i,8)==7)
			min_distance = 31;
			for j = 1:8
				dis_cal = sum(xor(seq_arr(j,:), demapping(1,:)));
				if (dis_cal < min_distance)
					min_distance = dis_cal;
				end
			end
			if (min_distance<3)
				sync = 1;
				seq_cnt = 0;
				decoded_seq = zeros(1,1);
				angle_ori = [];
				angle_deltaF = [];
				diff_angle_ori = [];
				diff_angle_deltaF = [];
			end
		end
	else % sync-ed
		if (seq_cnt==255)
			seq_cnt = 0;
			decoded_sym = 99;
			min_distance = 31;
			for j = 1:8
				for k = 1:16
					dis_cal = sum(xor(seq_arr(j,:), demapping(k,:)));
					if (dis_cal<min_distance)
						min_distance = dis_cal;
						decoded_sym = k-1;
					end
				end
			end

			packet_counter = packet_counter + 1;

			switch(state)
				case 0
					if (size(decoded_seq, 2)>=2)
						if ((decoded_seq(end)==10)&&(decoded_seq(end-1)==7))	%0xa7
							state = 1;
							packet_counter = 0;
						end
					end

				case 1
					if (packet_counter==2)
						rx_length = decoded_seq(end)*16 + decoded_seq(end-1);
						packet_length = 0;
						packet_counter = 0;
						state = 2;
					end

				case 2
					if (packet_counter==2)
						packet_length = packet_length + 1;
						packet_counter = 0;
						if (packet_length==rx_length)
							break;
						end
					end
			end
			% end of switch

			decoded_seq = [decoded_seq decoded_sym];
		
		else
			seq_cnt = seq_cnt + 1;
		end

		% down-sampling to 4MHz by dropping 3 samples out of 4
		if (mod(seq_cnt, 4)==0)
			angle_ori = [angle_ori atan2(rx_Q(i), rx_I(i))*180/pi];
			angle_deltaF = [angle_deltaF atan2(rx_Q_deltaF(i), rx_I_deltaF(i))*180/pi];

			if (size(angle_ori, 2)>1)
				temp = angle_ori(end) - angle_ori(end-1);
				if (temp<-180)
					temp = temp + 360;
				elseif (temp>180)
					temp = temp - 360;
				end
				diff_angle_ori = [diff_angle_ori temp];
			end

			if (size(angle_deltaF, 2)>1)
				temp = angle_deltaF(end) - angle_deltaF(end-1);
				if (temp<-180)
					temp = temp + 360;
				elseif (temp>180)
					temp = temp - 360;
				end
				diff_angle_deltaF = [diff_angle_deltaF temp];
			end

			% frequency compensate
			if (size(diff_angle_deltaF, 2)>2)
				diff1 = diff_angle_deltaF(end) - diff_angle_deltaF(end-1);
				diff2 = diff_angle_deltaF(end-1) - diff_angle_deltaF(end-2);

				if ( (sign(diff_angle_deltaF(end))==sign(diff_angle_deltaF(end-1))) && ... 
					(sign(diff_angle_deltaF(end))==sign(diff_angle_deltaF(end-2))) && (abs(diff1)<10) && (abs(diff2)<10))
					diff_arr = [diff_arr diff_angle_deltaF(end)];
					if (sign(diff_angle_deltaF(end))==1)	% positive
						if (diff_angle_deltaF(end)>45)
							freq_comp = freq_comp + freq_resolution;
						else
							freq_comp = freq_comp - freq_resolution;
						end
					else	% negative
						if (diff_angle_deltaF(end)<-45)
							freq_comp = freq_comp - freq_resolution;
						else
							freq_comp = freq_comp + freq_resolution;
						end
					end
				end
			end

		end
		% end of down sampling

	end
	% end of sync

end
out = deltaF + freq_comp;
%{
plot(diff_angle_ori);
hold
plot(diff_angle_deltaF, 'r');
grid on;
hold off;
%}
