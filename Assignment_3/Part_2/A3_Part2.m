%% Assignment 3 - Part 2

% Add handoutfiles to path
%addpath('/Users/paalthorseth/Documents/Git/TTK4190/Assignment_3/Handouts/Matlab');
addpath('../Handouts/Matlab');
%addpath('/Users/paalthorseth/Documents/Git/TTK4190/MSS-master');
addpath('../../MSS-master');
addpath('../Common')

dummy_values;


%% Initial conditions

p0=[1000,700]';     % Initial position (NED)
v0=[6.63 0]';       % Initial velocity (body)
psi0=60*pi/180;     % Inital yaw angle
r0=0;               % Inital yaw rate
c=0;                % Current on (1)/off (0)
delta_c=0;          % Constant reference, not used for this part

%% Heading autopilot

% Model parameters
T       = 130.7054;
K       = -0.0544;


% Heading control parameters, PID
zeta    = 1;        % Damping ratio
omega_n = 10*0.004; % Natural frequency

K_p_psi = T/K * omega_n^2; 
K_i_psi = omega_n/10 * K_p_psi;
K_d_psi = 1/K * (2*zeta*omega_n*T - 1);

%% Speed autopilot

% Speed control parameters, Feedback
k_reg   = 0.3445;
r_reg   = 1.3628;

%% Task 2.1

% Activate both controllers
dc_mode  = DC_MODE.CONTROLLER;
nc_mode  = NC_MODE.CONTROLLER;
ref_mode = REF_MODE.PATH_FOLLOWING;

% Simulate system
tstart = 0;        % Sim start time
tstop  = 10000;    % Sim stop time
tsamp  = 10;       % Sampling time for how often states are stored. 
                   %     (NOT ODE solver time step)
                   
sim MSFartoystyring 

pathplotter(p(:,1), p(:,2), psi, tsamp, 1, tstart, tstop, 0, WP);

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

% Surge
figure('rend','painters','pos',[10 10 750 400]);
subplot(1,2,1);
hold on;
plot(t, v(:,1),'b-');
plot(t, u_d, 'r--');
grid on;
hold off;
title('Surge performance with current, PID controllers', ...
    'Interpreter', 'latex');
xlabel('Time [s]', 'Interpreter', 'latex');
ylabel('Velocity [m/s]', 'Interpreter', 'latex');
legend('$u$', '$u_d$', 'Interpreter', 'latex');

subplot(1,2,2);
plot(t, u_tilde, 'm');
grid on;
hold off;
title('Surge error, PID controllers', ...
    'Interpreter', 'latex');
xlabel('Time [s]', 'Interpreter', 'latex');
ylabel('Velocity [m/s]', 'Interpreter', 'latex');
legend('$\tilde{u}$', 'Interpreter', 'latex');
                   
%% Task 2.2




