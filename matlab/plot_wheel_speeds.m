figure(1);
hold on;
title('Right wheel speed vs. samples');
ylabel('angular velocity (rad/s)');
xlabel('samples');
plot(cnt, r_raw);
plot(cnt, r_filt);
legend('raw', 'filtered');

figure(2);
hold on;
title('Left wheel speed vs. samples');
ylabel('angular velocity (rad/s)');
xlabel('samples');
plot(cnt, l_raw);
plot(cnt, l_filt);
legend('raw', 'filtered');

figure(3);
hold on;
title('Right wheel speed vs. samples');
ylabel('angular velocity (rad/s)');
xlabel('samples');
plot(cnt, r_speed_sp);
plot(cnt, r_filt);
legend('Setpoint', 'Actual');

figure(4);
hold on;
title('Left wheel speed vs. samples');
ylabel('angular velocity (rad/s)');
xlabel('samples');
plot(cnt, l_speed_sp);
plot(cnt, l_filt);
legend('Setpoint', 'Actual');
