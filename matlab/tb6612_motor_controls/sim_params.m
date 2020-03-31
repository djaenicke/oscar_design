Ts = 0.025;         % Sample time

L_Kp = 4.5;         % Proportional gain
L_Ki = 10.0;        % Integral gain
L_Kd = 0.3;         % Derivative gain

R_Kp =  4.5;        % Proportional gain
R_Ki =  12.0;       % Integral gain
R_Kd =  0.3;        % Derivative gain

L_Ke = 0.215;       % Voltage to speed constants
R_Ke = 0.255;

Min_V = 0.0;        % Minimum voltage for the motors to spin
Driver_Vdrop = 0.1; % Voltage drop across motor driver
Max_V = 6.0 + Driver_Vdrop;

alpha = 0.4;

