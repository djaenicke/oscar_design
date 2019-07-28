% Sample time
Ts = 0.025;

% Get the continuous time transfer function
s = tf('s');

% From system id. toolbox
G = 6798/(s^3 + (67.18*s^2) + (1703*s) + 6004);

% Discretize the transfer function
Gd = c2d(G,Ts);

R_Ke = 0.226;
