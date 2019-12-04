close all;
data = load("-ascii", 'cl_w_fft_w_filters.csv');
fft.sp = data(:,2)./1000;
fft.r_speed = data(:,3)./1000;
fft.l_speed = data(:,4)./1000;
fft.n = length(fft.sp);

data = load("-ascii", 'cl_period_meas_w_filters.csv');
period.sp = data(:,2)./1000;
period.r_speed = data(:,3)./1000;
period.l_speed = data(:,4)./1000;
period.n = length(period.sp);

N = min(fft.n, period.n);
t = 0:0.025:0.025*(N-1);

% Right Wheel Speed Plot
figure(1);
plot(t, fft.sp(1:N));
hold on;
plot(t, fft.r_speed(1:N), '-o', 'MarkerSize', 3)
plot(t, period.r_speed(1:N), 'g-*', 'MarkerSize', 3);
legend('Setpoint', 'FFT method', 'Period method');
title('Right Wheel Speed vs. Time');
ylabel('speed (rad/s)');
xlabel('time (s)');

% Left Wheel Speed Plot
figure(2);
plot(t, fft.sp(1:N));
hold on;
plot(t, fft.l_speed(1:N), '-o', 'MarkerSize', 3)
plot(t, period.l_speed(1:N), 'g-*', 'MarkerSize', 3);
legend('Setpoint', 'FFT method', 'Period method');
title('Left Wheel Speed vs. Time');
ylabel('speed (rad/s)');
xlabel('time (s)');

% Compute steady state variance
fft.r_var = var(fft.r_speed(30:N));
period.r_var = var(period.r_speed(30:N));

fft.l_var = var(fft.l_speed(30:N))
period.l_var = var(period.l_speed(30:N))


