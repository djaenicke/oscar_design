Ts = 0.025;         % Sample time

L_Kp = 1.95;        % Proportional gain
L_Ki = 3.90;        % Integral gain
L_Kd = 0.05;        % Derivative gain

R_Kp =  1.45;       % Proportional gain
R_Ki =  3.50;       % Integral gain
R_Kd =  0.05;       % Derivative gain

L_Ke = 0.215;       % Voltage to speed constants
R_Ke = 0.255;

Min_V = 0.0;        % Minimum voltage for the motors to spin
Driver_Vdrop = 0.1; % Voltage drop across motor driver
Max_V = 6.0 + Driver_Vdrop;

alpha = 0.2;

