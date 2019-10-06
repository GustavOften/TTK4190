clear; close all;

%% Assignment 2 Part 1

A2_model;

%% Problem 1

% Ground speed
V_g = V_a;

%Dutch roll mode
dutch_roll_matrix = [A(1,1) A(1,4); A(4,1) A(4,4)];
eigs(dutch_roll_matrix);

%Spiral-divergence mode
eig_spiral = (A(4,4)*A(3,1)-A(4,1)*A(3,4))/A(3,1);

%Roll mode
eig_roll = A(3,3);

%% Problem 2

%% 2.a) -----------------------------------------------------------

a_phi_1 = -0.65;
a_phi_2 = 2.87;

H_p_delta_a = tf([0 a_phi_1], [1 a_phi_2]);

%% 2.b) -----------------------------------------------------------

delta_a_max = 30;
e_phi_max = 15;
zeta_phi = 0.707;
zeta_chi = 0.9;
d = 1.5;
W_chi = 7;
omega_n_phi = sqrt(abs(a_phi_2) * delta_a_max/e_phi_max);
omega_n_chi = 1/W_chi * omega_n_phi;

% Roll control parameters
k_p_phi = delta_a_max/e_phi_max * sign(a_phi_2);                    
k_i_phi = 0.0;   
k_d_phi = 2 * zeta_phi * omega_n_phi/a_phi_2;

% Course control parameters
k_p_chi = 2 * zeta_chi * omega_n_chi * V_g/g;     
k_i_chi = omega_n_chi^2 * V_g/g;  

H_phi_phic = tf([ k_p_phi*a_phi_2          k_i_phi*a_phi_2 ], ...
                [     1             (a_phi_1 + a_phi_2*k_d_phi) ...
                  k_p_phi*a_phi_2          k_i_phi*a_phi_2 ]);

tf_root_analysis_k_i_phi = tf([ 0                        k_i_phi*a_phi_2 ], ...
                [     1             (a_phi_1 + a_phi_2*k_d_phi) ...
                  k_p_phi*a_phi_2          k_i_phi*a_phi_2 ]);

% Plot rlocus
rlocus(tf_root_analysis_k_i_phi)



%% 2.c) -----------------------------------------------------------


%% 2.d) -----------------------------------------------------------
sim_time = 10;
chi_control = timeseries(ones(sim_time,1));

out = sim('model_autopilot', sim_time);

figure('rend','painters','pos',[10 10 750 400])
hold on;
plot(out.chi)
title("Plot of chi");
xlabel("time[s]");
ylabel("chi[deg]");
grid on;
hold off;

figure('rend','painters','pos',[10 10 750 400])
hold on;
plot(chi_control)
title("Plot of chi control");
xlabel("time[s]");
ylabel("chi control[deg]");
grid on;
hold off;

figure('rend','painters','pos',[10 10 750 400])
hold on;
plot(out.delta_control)
title("Plot of delta control");
xlabel("time[s]");
ylabel("delta control[deg]");
grid on;
hold off;

%% 2.e) -----------------------------------------------------------
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

g = 9.81;


% Actuator dynamics
H_l = tf([0 7.5],[1 7.5]);




deg2rad = pi/180;   
rad2deg = 180/pi;


Ts = 0.01;
N = 10/Ts;
table = zeros(N+1,7); % tabel(i) = [beta  phi  p  r  delta_a  chi  t]
chi_c = ones(N+1,1);
chi_error_i = 0;

for i = 2:N+1
    t = (i-1)*Ts;
    chi_error = chi_c(i)-table(i-1,6); 
    chi_error_i = chi_error_i + chi_error*Ts;
    phi_c = chi_error*k_p_chi + chi_error_i*k_i_chi;
    u = (phi_c-table(i-1,2))*k_p_phi - table(i-1,3)*k_d_phi;
    if u < -30
       u = -30;
    
    elseif u > 30
       u = 30;
    end
    u = deg2rad*u;
    chi_dot = (g/V_a)*tan(table(i-1,2))*cos(table(i-1,1));
    table(i,1:5) = (A*table(i-1,1:5)' + B*u)*Ts + table(i-1,1:5)';
 
    
    table(i,6)= table(i-1,6) + chi_dot*Ts;
    table(i,7) = t;
end

beta    = rad2deg*table(:,1); 
phi     = rad2deg*table(:,2);
p       = rad2deg*table(:,3);
r       = rad2deg*table(:,4);
delta_a = rad2deg*table(:,5);
chi     = rad2deg*table(:,6);
time       = table(:,7);  



figure (1); clf;
hold on;
plot(time, chi, 'b');
plot(time, delta_a, 'r');
plot(time, phi, 'g');
hold off;
grid on;
legend('\chi', '\delta_a', '\phi');
title('This');
xlabel('time [s]'); 
ylabel('angle [deg]');