data = load("-ascii", 'open_loop_no_filter.csv');
speed_sp = data(:,2)./1000;
r_wheel_speed = data(:,3)./1000;
l_wheel_speed = data(:,4)./1000;