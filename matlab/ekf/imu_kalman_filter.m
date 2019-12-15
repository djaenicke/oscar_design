dt = 0.025;
num_states = 8;
process_noise = 0.005;
sensor_noise = 0.3;

xhat = zeros(num_states,1);
xk   = zeros(num_states,1);
% x1 = heading angle(rad)
% x2 = yaw rate (rad/s)
% x3 = longitudinal position     (m)
% x4 = longitudinal velocity     (m/s)
% x5 = longitudinal acceleration (m/s^2)
% x6 = lateral position          (m)
% x7 = lateral velocity          (m/s)
% x8 = lateral acceleration      (m/s^2)

F = [1 dt 0 0 0 0 0 0;
     0 1 0 0 0 0 0 0;
     0 0 1 dt (dt^2)/2 0 0 0;
     0 0 0 1 dt 0 0 0;
     0 0 0 0 1 0 0 0;
     0 0 0 0 0 1 dt (dt^2)/2;
     0 0 0 0 0 0 1 dt;
     0 0 0 0 0 0 0 1];
 
 Q = eye(num_states)*process_noise;
 Pk = eye(num_states);
 
 R = eye(num_states)*sensor_noise;
 
 %% Simulation Parameters
data = load("-ascii", 'cl_period_meas_w_filters.csv');
period.sp = data(:,2)./1000;
period.r_speed = data(:,3)./1000;
period.l_speed = data(:,4)./1000;
N = length(period.sp);
t = 0:0.025:0.025*(N-1);

%% Simulation
for i=1:length(t)
    %% Prediction Step
    xhat = F*xhat;
    Pk = (Fk*Pk*Fk')+Q';
end