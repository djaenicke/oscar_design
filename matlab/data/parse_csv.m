data = load("-ascii", 'cl_w_fft_w_filters.csv');
speed_sp = data(:,2)./1000;
r_wheel_speed = data(:,3)./1000;
l_wheel_speed = data(:,4)./1000;
v_batt = data(:,5)./1000;
