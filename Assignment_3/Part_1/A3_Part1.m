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
tstart = 0;         % Sim start time
tstop  = 10000;     % Sim stop time
tsamp  = 10;        % Sampling time for how often states are stored. (NOT ODE solver time step)
                
p0   = zeros(2,1);  % Initial position (NED)
v0   = [6.63 0]';   % Initial velocity (body)
psi0 = 0;           % Inital yaw angle
r0   = 0;           % Inital yaw rate
c    = 0;           % Current on (1)/off (0)

dc_mode = DC_MODE.CONSTANT;
nc_mode = NC_MODE.CONSTANT;

sim MSFartoystyring % The measurements from the simulink model are automatically written to the workspace.

% Heading rate, r(t)
fun     = @(x,xdata) r0*exp(-xdata/x(1)) + ...
            (1 - exp(-xdata/x(1)))*x(2)*(delta_c*180/pi);

% Defining data used for curve fitting
x0      = [50,0.1]';
xdata   = t;
ydata   = r*180/pi; % Heading rate in deg/s

% Nonlinear curve fitting using least squares
x       = lsqcurvefit(fun, x0, xdata, ydata);

% Plot
figure('rend','painters','pos',[10 10 750 400]);
times = linspace(xdata(1),xdata(end));
plot(xdata, ydata,'r--');
hold on;
plot(times,fun(x,times),'b-');
title('Least-squares fit of MS Fartoystyring, $\delta$ = $-5$ [deg]',...
    'interpreter','latex');
xlabel('Time [s]','interpreter','latex');
ylabel('Heading [deg]', 'interpreter', 'latex');
legend('Nonlinear model','Estimated 1st-order Nomoto model',...
    'interpreter','latex', 'location','southeast');

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
K_i_psi = omega_n/10 * K_p_psi;
K_d_psi = 1/K * (2*zeta*omega_n*T - 1);


% Simulate system
tstart = 0;        % Sim start time
tstop  = 10000;    % Sim stop time
tsamp  = 10;       % Sampling time for how often states are stored. 
                   %     (NOT ODE solver time step)
                
p0   = zeros(2,1); % Initial position (NED)
v0   = [6.63 0]';  % Initial velocity (body)
psi0 = 0;          % Inital yaw angle
r0   = 0;          % Inital yaw rate
c    = 1;          % Current on (1)/off (0)

dc_mode = DC_MODE.CONTROLLER;
nc_mode = NC_MODE.CONSTANT;

sim MSFartoystyring 


% Plot

% Heading
figure('rend','painters','pos',[10 10 750 400]);

subplot(1,2,1);
hold on;
plot(t, psi*180/pi,'b-');
plot(t, psi_d*180/pi, 'r-.');
grid on;
hold off;
title('Heading $\psi$ versus Heading reference $\psi_d$', ...
    'Interpreter', 'latex');
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
title('Heading rate $r$ versus Heading rate reference $r_d$', ...
    'Interpreter', 'latex');
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
plot(t, dc*180/pi,'r-');
plot(t, -25*ones(size(t)), 'k--');
plot(t, 25*ones(size(t)), 'k--');
ylim([-30,30]);
grid on;
hold off;
title('Rudder input $\delta_c$ with saturation limits', ...
    'Interpreter', 'latex');
xlabel('Time [s]', 'Interpreter', 'latex');
ylabel('Angle [deg]', 'Interpreter', 'latex');
legend('$\delta_c$', 'Interpreter', 'latex');

%% task 1.6

% Rudder reference
% Simulate system
tstart = 0;         % Sim start time
tstop  = 10000;     % Sim stop time
tsamp  = 10;        % Sampling time for how often states are stored
                
p0   = zeros(2,1);  % Initial position (NED)
v0   = [5 0]';      % Initial velocity (body)
psi0 = 0;           % Inital yaw angle
r0   = 0;           % Inital yaw rate
c    = 0;           % Current on (1)/off (0)

dc_mode = DC_MODE.CONTROLLER;
nc_mode = NC_MODE.STEP;

psi_d_amp = 0;
psi_d_bias = 0;
psi_d_freq = 0;

nc_step_time  = 0;
nc_step_start = 5;
nc_step_final = 8;

sim MSFartoystyring 

% start_index = floor((nc_step_time)/tsamp) + 1;
% end_index   = floor((tstop)/tsamp) + 1;
% indata      = nc(start_index:end_index);
% outdata     = v(start_index:end_index, 1);
% 
% iodata      = iddata(outdata, indata, tsamp);
% tfinfo      = tfest(iodata, 1);
% 
% vel_gain    = tfinfo.Numerator(1)/tfinfo.Denominator(2);
% vel_ts      = 1/tfinfo.Denominator(2);
% 
% u_sim       = lsim(tfinfo, nc, t);
% 
% % Plot
% figure('rend','painters','pos',[10 10 750 400]);
% plot(t, v(:,1),'r--');
% hold on;
% plot(t,u_sim,'b-')
% plot(t,nc,'k-.')
% title('Instrumental variable estimation fit of step response')
% xlabel('Time [s]')
% legend('Nonlinear model','Estimated 1st order model','Shaft input', ...
%     'location','southeast')
% grid on;
% xlim([4000 10000]);
% ylim([3.5 8.5]);

fun_surge = @(x,xdata) -nc_step_final*x(2)/x(1) + ...
    exp(x(1)*xdata)*(v0(1) + nc_step_final*x(2)/x(1));

% Defining data used for curve fitting
start_index   = floor(nc_step_time/tsamp) + 1;
x0_surge      = [-1,1]';
xdata_surge   = t(start_index:end_index) - nc_step_time;
ydata_surge   = v(start_index:end_index,1); % Heading rate in deg/s

% Nonlinear curve fitting using least squares
x_surge      = lsqcurvefit(fun_surge, x0_surge, xdata_surge, ydata_surge);

% Plot
figure();
times = linspace(xdata_surge(1),xdata_surge(end));
plot(xdata_surge, ydata_surge, 'r--');
hold on;
plot(times,fun_surge(x_surge,times),'b-')
title('Least-squares fit of MS Fartoystyring model for nc = 8 rad/s', ...
    'interpreter','latex')
xlabel('Time [s]','interpreter','latex')
xlabel('Velocity [m/s]','interpreter','latex')
legend('Nonlinear model','Estimated 1st order surge model', ...
    'location','southeast','interpreter','latex')
grid on;

%% task 1.7

% no simulation, only implementation
% pole placement of surge dynamics
k_reg = (1/300+x_surge(1))/x_surge(2);
r_reg = (1/300/x_surge(2));

%% task 1.8

% activate both controllers
dc_mode = DC_MODE.CONTROLLER;
nc_mode = NC_MODE.CONTROLLER;

% zero heading reference
psi_d_amp = 0;
psi_d_bias = 0;
psi_d_freq = 0;

% velocity step reference
u_d_step_time  = 1250;
u_d_step_start = 3;
u_d_step_final  = 8.3;

% initial conditions
v0   = [3 0]';        % Initial velocity (body)
psi0 = 0;             % Inital yaw angle
r0   = 0;             % Inital yaw rate

% simulation settings
c    = 1;

% run sim
sim MSFartoystyring

% plotting
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

figure('rend','painters','pos',[10 10 750 400]);
subplot(1,2,1);
plot(t, psi*180/pi,'r');
grid on;
hold off;
title('Heading, PID controllers', 'Interpreter', 'latex');
xlabel('Time [s]', 'Interpreter', 'latex');
ylabel('Angle [deg]', 'Interpreter', 'latex');
legend('$\psi$', 'Interpreter', 'latex');

subplot(1,2,2);
plot(t, r*180/pi, 'b');
grid on;
hold off;
title('Yaw-rate, PID controllers', 'Interpreter', 'latex');
xlabel('Time [s]', 'Interpreter', 'latex');
ylabel('Angle [deg/s]', 'Interpreter', 'latex');
legend('$r$', 'Interpreter', 'latex');


figure('rend','painters','pos',[10 10 750 400]);

subplot(1,2,1);
hold on;
plot(t, dc*180/pi,'r-');
plot([0 max(t)], [-25 -25], 'b--');
plot([0 max(t)], [25 25], 'b--');
xlabel('Time [s]', 'Interpreter', 'latex');
ylabel('Angle [deg]', 'Interpreter', 'latex');
legend('$\delta_c$', 'Interpreter', 'latex');
title('Rudder input, PID controllers', 'Interpreter', 'latex');
grid on;

subplot(1,2,2);
hold on;
plot(t, nc, 'r-');
plot([0 max(t)], [-(85*2*pi)/60 -(85*2*pi)/60], 'b--');
plot([0 max(t)], [(85*2*pi)/60 (85*2*pi)/60], 'b--');
grid on;
hold off;
xlabel('Time [s]', 'Interpreter', 'latex');
ylabel('Angle [deg, deg/s]', 'Interpreter', 'latex');
legend('$n_c$', 'Interpreter', 'latex');
title('Shaft input, PID controllers', 'Interpreter', 'latex');
