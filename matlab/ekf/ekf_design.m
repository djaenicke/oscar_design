num_states = 3;
dt = 0.025;  % update rate  (s)
r  = 0.0336; % wheel radius (m)
wb = 0.134;  % wheel base   (m)
encoder_variance = 0.008; % (rad/s)^2

xhat = zeros(num_states,1);
xk   = zeros(num_states,1);
% x1 = heading angle         (rad)
% x2 = longitudinal position (m)
% x3 = lateral position      (m)

Mt = zeros(2);
Vt = zeros(3,2);
Vt(3,2) = 1;

alpha1 = 0.2; % process noise
alpha2 = 0.3; % process noise
alpha3 = 0.9; % process noise
alpha4 = 0.8; % process noise

u = zeros(2,1); % control vector
% u1 = right wheel angular velocity command (rad/s)
% u2 = left wheel angular velocity command (rad/s)

Fk = eye(num_states);
Pk = eye(num_states);

j = 1;
Hk = zeros(j,num_states);
Hk(1,1) = wb;

Rk = eye(j)*encoder_variance; %Covariance of sensor noise (jxj)
vk = zeros(j, j); % observation noise (jx1)

%% Simulation Parameters
data = load("-ascii", 'cl_period_meas_w_filters.csv');
period.sp = data(:,2)./1000;
period.r_speed = data(:,3)./1000;
period.l_speed = data(:,4)./1000;
N = length(period.sp);
t = 0:0.025:0.025*(N-1);

xout = zeros(3,N);

%% Simulation
for i=1:length(t)
    %% Prediction Step
    u(1) = period.sp(i);
    u(2) = period.sp(i);
    
    V = r/2*(u(1)+u(2)); % linear velocity
    W = (u(1)-u(2))/wb;  % rotational velocity
    
    phi = xhat(1)+(W*dt/2);
    Fk(2,1) = dt*V*-sin(phi);
    Fk(3,1) = dt*V*cos(phi);
    
    % Compute f
    xhat(1) = xhat(1) + dt*W;
    xhat(2) = xhat(2) + dt*V*cos(phi);
    xhat(3) = xhat(3) + dt*V*sin(phi);
    
    % Update process noise
    Mt(1,1) = (alpha1*V^2)+(alpha2*W^2);
    Mt(2,2) = (alpha3*V^2)+(alpha4*W^2);
    
    % Map Mt from control space to state space using Vt
    Vt(1,1) = cos(phi);
    Vt(1,2) = -sin(phi)/2;
    Vt(2,1) = sin(phi);
    Vt(2,2) = cos(phi)/2;
    
    % Compute a priori covariance matrix
    Pk = Fk*Pk*Fk'+Vt*Mt*Vt';
    
    %% Update Step
    xk(1) = (period.r_speed(i) - period.l_speed(i));
    zk = Hk*xk + vk;
    K = Pk*Hk'*((Hk*Pk*Hk')+Rk)^-1;
    xhat = xhat + (K*(zk-(Hk*xhat)));
    
    xout(:,i) = xhat;
end

plot(xout(1,:))
hold on
plot((period.r_speed - period.l_speed(i)).*(dt/wb));
legend('EKF heading', 'org heading');
  