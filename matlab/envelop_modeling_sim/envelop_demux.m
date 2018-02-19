
function [out, iq_rand_cnt, numOfErrBytes] = envelop_demux(snr, num_of_dat, deltaF1, deltaF2)

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


%phi = round(rand()*360);
%phi = phi/180*pi;
phi = 0;

val(1) = 0;
seq_arr = zeros(8,31);
val_integ(1:7) = zeros(1, 7);
decoded_bit(1:7) = zeros(1, 7);
sync = 0;
seq_cnt = 0;
state = 0;
packet_length = 127;
packet_counter = 0;
rx_length = 0;

rx_I_envelop = zeros(1, size(t,2));
rx_Q_envelop = zeros(1, size(t,2));

amplitude = 256;
artificial_thres = round(0.05*amplitude);
icnt = 0;
qcnt = 0;

for i = 1:size(rx_I, 2)

	rx_I_envelop(i) = cos(2*pi*((deltaF1-deltaF2)/2)*i*T_sample).*(rx_I(i)*cos(2*pi*((deltaF1+deltaF2)/2)*i*T_sample+ phi) + rx_Q(i)*sin(2*pi*((deltaF1+deltaF2)/2)*i*T_sample+ phi));
	rx_Q_envelop(i) = cos(2*pi*((deltaF1-deltaF2)/2)*i*T_sample).*(rx_Q(i)*cos(2*pi*((deltaF1+deltaF2)/2)*i*T_sample+ phi) - rx_I(i)*sin(2*pi*((deltaF1+deltaF2)/2)*i*T_sample+ phi));
	
	if ((rx_I_envelop(i)<artificial_thres)&&(rx_I_envelop(i)>-1*artificial_thres))
		rx_I_envelop(i) = round(rand()*2*artificial_thres) - artificial_thres;
		icnt = icnt + 1;
	end

	if ((rx_Q_envelop(i)<artificial_thres)&&(rx_Q_envelop(i)>-1*artificial_thres))
		rx_Q_envelop(i) = round(rand()*2*artificial_thres - artificial_thres);
		qcnt = qcnt + 1;
	end

	% demux
	if (i>1)
		val(i) = rx_Q_envelop(i)*rx_I_envelop(i-1) - rx_Q_envelop(i-1)*rx_I_envelop(i);
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

	end
	% end of sync
end

iq_rand_cnt = (icnt + qcnt)/2;
numOfErrBytes = 0;
if (size(decoded_seq,2)<2*(packet_length+2)+1)
	out = 1;
else
	temp1 = xor(decoded_seq(end-2*(packet_length+2)+1:end), gen_seq(end-2*(packet_length+2)+1:end));
	numOfErrBytes = sum(temp1);
	if (numOfErrBytes >0)
		out = 1;
	else
		out = 0;
	end
end

%debugging information
%{
if (out==1)
	temp2 = [sync, temp];
	disp(temp2);
	display(decoded_seq);
	display(gen_seq);
	display(temp1);
plot(t, rx_I_envelop, 'r', t, rx_Q_envelop, 'b');
grid on;
pause;
end
%}

