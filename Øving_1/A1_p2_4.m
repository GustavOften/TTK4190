% Parameters
T = 20; % Nomoto time constant
k = 0.1; % Nomoto gain

b = 0.001; % Constant bias

U = 5; % Assume speed-controller able to keep the total speed at 5m/s

% Initial conditions
x_0 = 0;
y_0 = 100;
psi_0 = 0;
r_0 = 0;

% PID-controller gains
k_p = 1;
k_d = 1;
k_i = 0.1;