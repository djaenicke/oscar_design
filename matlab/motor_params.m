J = 0.00051;
b = 0.003;
Ke = 0.220264317;
Kt = 0.143171806;
R = 3.3;
L = 0.076;

% Sample time
Ts = 0.050;

% Get the continuous time transfer function
s = tf('s');
G = Kt/((J*s+b)*(L*s+R)+(Ke*Kt));

% Discretize the transfer function
Gd = c2d(G,Ts);
