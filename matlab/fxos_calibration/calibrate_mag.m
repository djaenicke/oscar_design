data1 = load("-ascii", 'fxos_mag_cal_data1.csv');
data2 = load("-ascii", 'fxos_mag_cal_data2.csv');
data3 = load("-ascii", 'fxos_mag_cal_data3.csv');

D = zeros(size(data1, 1) + size(data2, 1) + size(data3, 1), 3);

D(:,1) = [data1(:,1); data2(:,1); data3(:,1)] .* 0.1; % Bx
D(:,2) = [data1(:,2); data2(:,2); data3(:,2)] .* 0.1; % Bx
D(:,3) = [data1(:,3); data2(:,3); data3(:,3)] .* 0.1; % Bx

% Visualize the uncalibrated and calibrated magnetometer data.
figure(1)
plot3(D(:,1),D(:,2),D(:,3), 'LineStyle', 'none', 'Marker', 'X', ...
    'MarkerSize', 8);
hold on;

% Compute the hard and soft iron corrections
[A,B,EXPMFS] = magcal(D);
F = (D-B)*A;

% Visualize the uncalibrated and calibrated magnetometer data.
plot3(F(:,1),F(:,2),F(:,3), 'LineStyle', 'none', 'Marker', 'X', ...
    'MarkerSize', 8);

