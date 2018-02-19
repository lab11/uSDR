module frequency_table(channels, freq_upper_bound, freq_lower_bound, freq_center);

input	[3:0]	channels;
output	[23:0]	freq_upper_bound, freq_lower_bound, freq_center;
reg		[23:0]	freq_upper_bound, freq_lower_bound, freq_center;
//Frequency Tolerance: 100000
always @ *
begin
	case(channels)
		0:
		begin
			freq_center = 24'd7880704;
			freq_upper_bound = 24'd7881032;
			freq_lower_bound = 24'd7880376;
		end

		1:
		begin
			freq_center = 24'd7897088;
			freq_upper_bound = 24'd7897416;
			freq_lower_bound = 24'd7896760;
		end

		2:
		begin
			freq_center = 24'd7913472;
			freq_upper_bound = 24'd7913800;
			freq_lower_bound = 24'd7913144;
		end

		3:
		begin
			freq_center = 24'd7929856;
			freq_upper_bound = 24'd7930184;
			freq_lower_bound = 24'd7929528;
		end

		4:
		begin
			freq_center = 24'd7946240;
			freq_upper_bound = 24'd7946568;
			freq_lower_bound = 24'd7945912;
		end

		5:
		begin
			freq_center = 24'd7962624;
			freq_upper_bound = 24'd7962952;
			freq_lower_bound = 24'd7962296;
		end

		6:
		begin
			freq_center = 24'd7979008;
			freq_upper_bound = 24'd7979336;
			freq_lower_bound = 24'd7978680;
		end

		7:
		begin
			freq_center = 24'd7995392;
			freq_upper_bound = 24'd7995720;
			freq_lower_bound = 24'd7995064;
		end

		8:
		begin
			freq_center = 24'd8011776;
			freq_upper_bound = 24'd8012104;
			freq_lower_bound = 24'd8011448;
		end

		9:
		begin
			freq_center = 24'd8028160;
			freq_upper_bound = 24'd8028488;
			freq_lower_bound = 24'd8027832;
		end

		10:
		begin
			freq_center = 24'd8044544;
			freq_upper_bound = 24'd8044872;
			freq_lower_bound = 24'd8044216;
		end

		11:
		begin
			freq_center = 24'd8060928;
			freq_upper_bound = 24'd8061256;
			freq_lower_bound = 24'd8060600;
		end

		12:
		begin
			freq_center = 24'd8077312;
			freq_upper_bound = 24'd8077640;
			freq_lower_bound = 24'd8076984;
		end

		13:
		begin
			freq_center = 24'd8093696;
			freq_upper_bound = 24'd8094024;
			freq_lower_bound = 24'd8093368;
		end

		14:
		begin
			freq_center = 24'd8110080;
			freq_upper_bound = 24'd8110408;
			freq_lower_bound = 24'd8109752;
		end

		15:
		begin
			freq_center = 24'd8126464;
			freq_upper_bound = 24'd8126792;
			freq_lower_bound = 24'd8126136;
		end

	endcase
end
endmodule
