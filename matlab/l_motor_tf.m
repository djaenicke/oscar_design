% Sample time
Ts = 0.025;

% Get the continuous time transfer function
s = tf('s');

% From system id. toolbox
G = 3421/(s^3 + (57.24*s^2) + (1283*s) + 4300);

% Discretize the transfer function
Gd = c2d(G,Ts);

L_Ke = 0.192;
