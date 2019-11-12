%% Assignment 3 - Part 1

% Add handoutfiles to path
%addpath('/Users/paalthorseth/Documents/Git/TTK4190/Assignment_3/Handouts/Matlab');
addpath('../Handouts/Matlab');
%addpath('/Users/paalthorseth/Documents/Git/TTK4190/MSS-master');
addpath('../../MSS-master');
addpath('../Common')

dummy_values;

%% Task 1.2

% Rudder reference
delta_c = -5*pi/180;

% Simulate system
tstart=0;           % Sim start time
tstop=10000;        % Sim stop time
tsamp=10;           % Sampling time for how often states are stored. (NOT ODE solver time step)
                
p0=zeros(2,1);      % Initial position (NED)
v0=[6.63 0]';       % Initial velocity (body)
psi0=0;             % Inital yaw angle
r0=0;               % Inital yaw rate
c=0;                % Current on (1)/off (0)
dc_mode = DC_MODE.CONSTANT;
sim MSFartoystyring % The measurements from the simulink model are automatically written to the workspace.

% Heading rate, r(t)
fun     = @(x,xdata)(r0*exp(-xdata/x(1)) + (1 - exp(-xdata/x(1)))*x(2)*(delta_c*180/pi));

% Defining data used for curve fitting
x0      = [50,0.1]';
xdata   = t;
ydata   = r*180/pi; % Heading rate in deg/s

% Nonlinear curve fitting using least squares
x       = lsqcurvefit(fun, x0, xdata, ydata);

% Plot
figure(1);
times = linspace(xdata(1),xdata(end));
plot(xdata, ydata,'ko',times,fun(x,times),'b-');
title('Nonlinear least-squares fit of MS Fartøystyring model for \delta = -5 (deg)')
xlabel('time (s)')
legend('Nonlinear model','Estimated 1st-order Nomoto model')

% Model parameters
T       = x(1);
K       = x(2);

%% Task 1.4

% Model parameters
T       = 130.7054;
K       = -0.0544;


% Heading control parameters
zeta    = 1;        % Damping ratio
omega_n = 10*0.004; % Natural frequency

K_p_psi = T/K * omega_n^2; 
K_i_psi = omega_n/10 * K_p;
K_d_psi = 1/K * (2*zeta*omega_n*T - 1);


% Simulate system
tstart=0;           % Sim start time
tstop=10000;        % Sim stop time
tsamp=10;           % Sampling time for how often states are stored. (NOT ODE solver time step)
                
p0=zeros(2,1);      % Initial position (NED)
v0=[6.63 0]';       % Initial velocity (body)
psi0=0;             % Inital yaw angle
r0=0;               % Inital yaw rate
c=1;                % Current on (1)/off (0)
dc_mode = DC_MODE.CONTROLLER;
sim MSFartoystyring % The measurements from the simulink model are automatically written to the workspace.


% Plot

% Heading
figure('rend','painters','pos',[10 10 750 400]);

subplot(1,2,1);
hold on;
plot(t, psi*180/pi,'b-');
plot(t, psi_d*180/pi, 'r-.');
grid on;
hold off;
title('Heading $\psi$ versus Heading reference $\psi_d$', 'Interpreter', 'latex');
xlabel('Time [s]', 'Interpreter', 'latex');
ylabel('Angle [deg]', 'Interpreter', 'latex');
legend('$\psi$', '$\psi_d$', 'Interpreter', 'latex');

subplot(1,2,2);
hold on;
plot(t, tilde_psi*180/pi,'k-');
grid on;
hold off;
title('Heading error $\tilde{\psi}$', 'Interpreter', 'latex');
xlabel('Time [s]', 'Interpreter', 'latex');
ylabel('Angle [deg]', 'Interpreter', 'latex');
legend('$\tilde{\psi}$', 'Interpreter', 'latex');

% Heading rate
figure('rend','painters','pos',[10 10 750 400]);

subplot(1,2,1);
hold on;
plot(t, r*180/pi,'b-');
plot(t, r_d*180/pi, 'r-.');
grid on;
hold off;
title('Heading rate $r$ versus Heading rate reference $r_d$', 'Interpreter', 'latex');
xlabel('Time [s]', 'Interpreter', 'latex');
ylabel('Angle [deg]', 'Interpreter', 'latex');
legend('$r$', '$r_d$', 'Interpreter', 'latex');

subplot(1,2,2);
hold on;
plot(t, tilde_r*180/pi,'k-');
grid on;
hold off;
title('Heading rate error $\tilde{r}$', 'Interpreter', 'latex');
xlabel('Time [s]', 'Interpreter', 'latex');
ylabel('Angle [deg]', 'Interpreter', 'latex');
legend('$\tilde{r}$', 'Interpreter', 'latex');

% Rudder input
figure('rend','painters','pos',[10 10 750 400]);

hold on;
plot(t, delta_c_PID*180/pi,'r-');
plot(t, -25*ones(size(t)), 'g-.');
plot(t, 25*ones(size(t)), 'g-.');
ylim([-30,30]);
grid on;
hold off;
title('Rudder input $\delta_c$ with saturation limits', 'Interpreter', 'latex');
xlabel('Time [s]', 'Interpreter', 'latex');
ylabel('Angle [deg]', 'Interpreter', 'latex');
legend('$\delta_c$', 'Interpreter', 'latex');

%% task 1.5



