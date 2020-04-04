data = load("-ascii", 'data.csv');
ts = data(:,1);
speed_sp = data(:,2);
r_wheel_speed = data(:,3);
l_wheel_speed = data(:,4);
v_batt = data(:,5);
r_wheel_dc = data(:,6);
l_wheel_dc = data(:,7);

figure(1);
plot(l_wheel_speed);
hold on;
plot(r_wheel_speed);
legend('left', 'right');
