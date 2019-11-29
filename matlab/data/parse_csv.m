data = load("-ascii", 'closed_loop_no_vbatt_filter_ss.csv');
speed_sp = data(:,2)./1000;
r_wheel_speed = data(:,3)./1000;
l_wheel_speed = data(:,4)./1000;
v_batt = data(:,5)./1000;
