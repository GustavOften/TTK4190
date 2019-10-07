%% Clean-up
clear; clear all;

%% Nifty bits
deg2rad = pi/180;
rad2deg = 180/pi;

%% State space
A = [-0.322 0.052 0.028 -1.12 0.002;
    0 0 1 -0.001 0;
    -10.6 0 -2.87 0.46 -0.65;
    6.87 0 -0.04 -0.32 -0.02;
    0 0 0 0 -7.5];

B = [0; 0; 0; 0; 7.5];

C = [1 0 0 0 0;
    0 1 0 0 0;
    0 0 1 0 0;
    0 0 0 1 0];

W = [0.052; 0; 0; 0; 0];

D = 0;

%% System parameters
%Some other constants
V_a = 580/3.6;
aileron_sat = 30*deg2rad; %[deg]
g = 9.81;

%Inner loop transfer function parameters (B&C figure 6.9)
a_2 = -0.65;
a_1 = 2.87;

u_max = 30; %[deg]
e_max = 15; %[deg]

omega_phi = sqrt(abs(a_2)*(u_max/e_max));
seta_phi = 0.707;

k_p_phi = -(u_max/e_max);
k_d_phi = (2*seta_phi*omega_phi - a_1)/a_2;
k_i_phi = -0.000; %By observing root-locus-curves, see second_task.m

omega_chi = omega_phi/20;
seta_chi = 1; %Chosen for niceness

k_p_chi = 2*seta_chi*omega_chi*V_a/g;
k_i_chi = (omega_chi^2)*V_a/g;

%% Simulation parameters
sim_time    = 800;
Ts           = 0.01;
N           = sim_time/Ts;
time_vec    = (0:Ts:sim_time-Ts);

%% Allocate state vectors
%Specify some initial-values
chi_0       = 0;
p_0         = 0;
phi_0       = 0;
beta_0      = 0;
r_0         = 0;
delta_0     = 0;

chi         = zeros(1, N); chi(1) = chi_0;
p           = zeros(1, N); p(1) = p_0;
phi         = zeros(1, N); phi(1) = phi_0;
beta        = zeros(1,N); beta(1) = beta_0;
r           = zeros(1,N); r(1) = r_0;
delta       = zeros(1,N); delta(1) = delta_0;
e_chi_int   = zeros(1,N);
e_phi_int   = zeros(1,N);

x = zeros(5,N);
x(:,1) = [beta_0; phi_0; p_0; r_0; delta_0];

delta_a     = zeros(1, N);
chi_c     = zeros(1, N);
chi_c(N/4:N/2) = 15*deg2rad;
chi_c(N/2:5*N/8) = 5*deg2rad;
chi_c(5*N/8:6*N/8) = 10*deg2rad;
d           = 1.5*deg2rad;

%% Simulation loop
for i = 1:N
    t = (i-1)*Ts;
    chi_error = chi_c(i) - chi(i);
    phi_c = k_i_chi*e_chi_int(i) + k_p_chi*chi_error;
    e_phi = phi_c - x(2,i);
    
    delta_a(i) = k_i_phi*e_phi_int(i) + k_p_phi*e_phi - k_d_phi*x(3,i);
    if delta_a(i) >= aileron_sat
            delta_a(i) = aileron_sat;
    elseif delta_a(i) <= -aileron_sat
            delta_a(i) = -aileron_sat;
    end
    
    if i < N
        
        x(:,i+1) = (eye(5) + A*Ts)*x(:,i) + B*Ts*delta_a(i) + W*Ts*d;
        chi(i+1) = chi(i) + Ts*(g/V_a)*tan(x(2,i) + d)*cos(x(1,i));

        e_chi_int(i+1) = e_chi_int(i) + chi_error*Ts;
        e_phi_int(i+1) = e_phi_int(i) + e_phi*Ts;
    end
end

%% Plot
chi = chi*rad2deg;
chi_c = chi_c*rad2deg;
figure (1);
hold on;
plot(time_vec, chi, 'b');
plot(time_vec, chi_c, 'r--');
hold off;
grid on;
title('Course');
xlabel('Time [s]'); 
ylabel('Course [deg]');


delta_a = delta_a*rad2deg;
figure(2)
hold on;
plot(time_vec, delta_a, 'b');
hold off;
grid on;
title('Input');
xlabel('Time [s]'); 
ylabel('Aileron [deg]');