%% Assignment 3 - Part 1

% Add handoutfiles to path
addpath('/Users/paalthorseth/Documents/Git/TTK4190/Assignment_3/Handouts/Matlab');


%% Task 1.2

K_p = 0;
K_d = 0;
K_i = 0;

% Rudder reference
delta_c = 15;

% Simulate system
run;

% Initial heading rate
r0 = 0;

% Heading rate, r(t)
r_fun   = @(x,xdata)(r0*exp(-xdata/x(1)) + (1 - exp(-xdata/x(1))*x(2)*delta_c));


% Nonlinear curve fitting using least squares
x0      = [1,1]';
x       = lsqcurvefit(r_fun, x0, t, r);

% Plotting solution
times = linspace(t(1),t(end));
plot(t,r,'ko',times,r_fun(x,times),'b-');
legend('Data','Fitted exponential');
title('Data and Fitted Curve');

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

% run;