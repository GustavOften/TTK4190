%% Assignment 3 - Part 2

% Adding paths, must be run from part folder
addpath('../Handouts/Matlab');
addpath('../../MSS-master');
addpath('../Common')

dummy_values;

%% Initial conditions

p0=[1000,700]';     % Initial position (NED)
v0=[6.63 0]';       % Initial velocity (body)
psi0=60*pi/180;     % Inital yaw angle
r0=0;               % Inital yaw rate
c=1;                % Current on (1)/off (0)

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

%% Load waypoints

load('WP.mat');
                   
%% Task 2.2

% Activate both controllers
dc_mode  = DC_MODE.CONTROLLER;
nc_mode  = NC_MODE.CONTROLLER;
ref_mode = REF_MODE.PATH_FOLLOWING_1;

% Simulate system
tstart = 0;        % Sim start time
tstop  = 10000;    % Sim stop time
tsamp  = 10;       % Sampling time for how often states are stored. 
                   %     (NOT ODE solver time step)
                   
% sim MSFartoystyring 

%% Task 2.6

% Activate both controllers
dc_mode  = DC_MODE.CONTROLLER;
nc_mode  = NC_MODE.CONTROLLER;
ref_mode = REF_MODE.PATH_FOLLOWING_2;

% Simulate system
tstart = 0;        % Sim start time
tstop  = 10000;    % Sim stop time
tsamp  = 10;       % Sampling time for how often states are stored. 
                   %     (NOT ODE solver time step)
                   
%sim MSFartoystyring

%% Plot closed-loop behaviour

%pathplotter(p(:,1), p(:,2), psi, tsamp, 1, tstart, t(end), 0, WP);

%% Task 2.4 and 2.6

% Course
%chi     = psi + beta;

% Plot
% figure('rend','painters','pos',[10 10 750 400]);
% hold on;
% plot(t, psi*180/pi,'b-.');
% plot(t, chi*180/pi,'g-');
% plot(t, chi_d*180/pi, 'r-');
% plot(t, beta*180/pi, 'm-');
% grid on;
% hold off;
% title('Heading $\psi$ vs Course $\chi$ vs Course reference $\chi_d$ vs Crab angle $\beta$', 'Interpreter', 'latex');
% xlabel('Time [s]', 'Interpreter', 'latex');
% ylabel('Angle [deg]', 'Interpreter', 'latex');
% legend('$\psi$', '$\chi$','$\chi_d$','$\beta$', 'Interpreter', 'latex');

%% Task 2.7

dc_mode  = DC_MODE.CONTROLLER;
nc_mode  = NC_MODE.CONTROLLER;
ref_mode = REF_MODE.TARGET_TRACKING;



tstop  = 5000;        % Sim stop time

U_T = 3;              % Target speed [m/s]
T_x0 = WP(1,2);
T_y0 = WP(2,2);

T_alpha   = atan2(WP(2,2)- WP(2,1), WP(1,2)- WP(1,1)); % Target travel direction
T_xdot = U_T*cos(T_alpha);                             % Target x velocity
T_ydot = U_T*sin(T_alpha);                             % Target y velocity

sim MSFartoystyring 
pathplotter(p(:,1), p(:,2), psi, tsamp, 1, tstart, tstop, 1, WP);

figure
plot(t,e);
title('Cross-track Error');
xlabel('Time [s]');
ylabel('distance [m]');

