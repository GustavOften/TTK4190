%% Aircraft Model


% Statespace matrices
A = [ -0.322    0.052   0.028   -1.12   0.002;
         0        0       1     -0.001    0;
      -10.6       0     -2.87    0.46   -0.65;
       6.87       0     -0.04   -0.32   -0.02;
         0        0       0        0     -7.5; ];


B = [ 0  0  0  0  7.5]';


C = [ 1 0 0 0 0
      0 1 0 0 0
      0 0 1 0 0
      0 0 0 1 0 ];
  
  
% Airspeed [km/h]
V_a = 580; 

% Gravity constant
g = 9.81;

% Actuator dynamics
H_l = tf([0 7.5],[1 7.5]);