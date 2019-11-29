Ts = 0.025;         % Sample time

L_Kp = 10.0;        % Proportional gain
L_Ki = 40.0;        % Integral gain
L_Kd = 0.52;        % Derivative gain

R_Kp =  5.5;        % Proportional gain
R_Ki =  15.0;       % Integral gain
R_Kd =  0.3;        % Derivative gain

L_Ke = 0.192;       % Voltage to speed constants
R_Ke = 0.226;

Vbatt = 8.5;        % Current battery voltage
Min_V = 3;          % Minimum voltage for the motors to spin
Driver_Vdrop = 2.0; % Voltage drop across motor driver
Max_V = Vbatt - Driver_Vdrop;

s = tf('s');

% Left motor continuous T.F. found using system id. toolbox
Gl = 3421/(s^3 + (57.24*s^2) + (1283*s) + 4300);

% Discretized left motor transfer function
Gld = c2d(Gl,Ts);

% Right motor continuous T.F. found using system id. toolbox
Gr = 6798/(s^3 + (67.18*s^2) + (1703*s) + 6004);

% Discretized right motor transfer function
Grd = c2d(Gr,Ts);

