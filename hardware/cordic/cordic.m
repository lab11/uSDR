function [out] = cordic(x, y)

num_of_stages = 10;
num_of_mantissa = 7; % 7 bits for mantissa

% initialize rom
angle_rom = zeros(1, num_of_stages);
angle_vrom = [];
for i = 1:num_of_stages
	angle_rom(i) = atan(2^(-(i-1)))*180/pi;
	angle_vrom = [angle_vrom; dec2bin(floor(angle_rom(i)*(2^num_of_mantissa)), 6+num_of_mantissa)];
end

% rotating vector
if (x<0)
	x_ori = -x;
else
	x_ori = x;
end

y_ori = y;
rotated_angle = 0;
for i = 1:num_of_stages
	if (y_ori>0)
		direction = -1; % clockwise
	else
		direction = 1;	% counter clockwise
	end
	x_new = x_ori - direction*2^(-(i-1))*y_ori;
	y_new = direction*2^(-(i-1))*x_ori + y_ori;
	rotated_angle = rotated_angle - direction*angle_rom(i);
	x_ori = x_new;
	y_ori = y_new;
end

if (x<0)
	if (y>0)
		rotated_angle = 180 - rotated_angle;
	else
		rotated_angle = -180 - rotated_angle;
	end
end

%display(angle_vrom);

out = rotated_angle;
