%% Assignment 3 - Part 1

% Add handoutfiles to path
addpath('/Users/paalthorseth/Documents/Git/TTK4190/Assignment_3/Handouts/Matlab');


%% Task 1.2

% Model parameters
K       = 1;
T       = 1;

%% Task 1.3


%% Task 1.4

% Heading control parameters
zeta    = 1;        % Damping ratio
omega_n = 10*0.004; % Natural frequency

K_p     = T/K * omega_n^2; 
K_i     = omega_n/10 * K_p;
K_d     = 1/K * (2*zeta*omega_n*T - 1);



%% Run simulation

run;