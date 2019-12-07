WHEEL_RADIUS = 0.0336;
data = load("-ascii", 'gtp_data_kp_5.csv');

cnt  = data(:,1);

fb.x = data(:,2);
fb.y = data(:,3);
fb.theta = data(:,4);

wr = data(:,5)./WHEEL_RADIUS;
wl = data(:,6)./WHEEL_RADIUS;

d = sqrt(fb.x.^2 + fb.y.^2);

figure(1);
plot(fb.theta);
legend('fb.theta');
title('Heading Angle vs. Samples');

figure(2);
plot(fb.x);
hold on;
plot(fb.y);
legend('fb.x', 'fb.y');

figure(3);
plot(d)
title('Distance traveled vs. Samples');

figure(4);
plot(fb.theta);
hold on;
plot(d)
legend('fb.theta', 'd');

