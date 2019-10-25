clear; close all;

%% Assignment 2 Part 2

addpath('/Users/paalthorseth/Documents/Git/TTK4190/Assignment_2');
A2_model;

%% Parameters

% Ground speed
V_g = V_a;

a_phi_2 = -0.65;
a_phi_1 = 2.87;

H_p_delta_a = tf([0 a_phi_2], [1 a_phi_1]);

% Parameters
delta_a_max = 30;
e_phi_max = 15;
zeta_phi = 0.707;
zeta_chi = 0.9; % was 0.9
d = 1.5*pi/180;
W_chi = 15; % was 7
omega_n_phi = sqrt(abs(a_phi_2) * delta_a_max/e_phi_max);
omega_n_chi = 1/W_chi * omega_n_phi;

% Roll control parameters
k_p_phi = delta_a_max/e_phi_max * sign(a_phi_2);                    
k_i_phi = 0.0;   
k_d_phi = (2 * zeta_phi * omega_n_phi - a_phi_1)/a_phi_2;

% Course control parameters
k_p_chi = 2 * zeta_chi * omega_n_chi * V_g/g;     
k_i_chi = omega_n_chi^2 * V_g/g;  


%% 3.e) -----------------------------------------------------------

sim_time = 500;

out = sim('A2_Part3_simulink_model', sim_time);

figure('rend','painters','pos',[10 10 750 400])
hold on;
plot(out.chi_control, "b");
plot(out.chi, "r");
title("Chi and chi_control");
xlabel("time [s]");
ylabel("angle [deg]");
grid on;
hold off;
legend({"\chi_c [deg]", "\chi [deg]"}, "Location", "northeast");

figure('rend','painters','pos',[10 10 750 400])
hold on;
plot(out.delta_control, "b");
title("Aileron input");
xlabel("time [s]");
ylabel("angle [deg]");
grid on;
hold off;
legend({"\delta_c [deg]", "\chi_c [deg]"}, "Location", "northeast");

figure('rend','painters','pos',[10 10 750 400])
hold on;
plot(out.beta, "b");
plot(out.beta_hat, "r");
title("Estimated sideslip and actual sideslip");
xlabel("time [s]");
ylabel("angle [deg]");
ylim([-0.5 0.5]);
grid on;
hold off;
legend({"\beta [deg]", "\beta_{estimated} [deg]"}, "Location", "northeast");

figure('rend','painters','pos',[10 10 750 400])
hold on;
plot(out.phi, "b");
plot(out.phi_hat, "r");
title("Estimated roll angle and actual roll angle");
xlabel("time [s]");
ylabel("angle [deg]");
ylim([-50 50]);
grid on;
hold off;
legend({"\phi [deg]", "\phi_{estimated} [deg]"}, "Location", "northeast");

figure('rend','painters','pos',[10 10 750 400])
hold on;
plot(out.p_measured, "g");
plot(out.p_hat, "r");
plot(out.p, "b");
title("Estimated, actual and measured roll rate");
xlabel("time [s]");
ylabel("angle [deg]");
%ylim([-0.5 0.5]);
grid on;
hold off;
legend({"p_{measured} [deg]", "p_{estimated} [deg]", "p [deg]", }, "Location", "northeast");

figure('rend','painters','pos',[10 10 750 400])
hold on;
plot(out.r_measured, "g");
plot(out.r_hat, "r");
plot(out.r, "b");
title("Estimated, actual and measured yaw rate");
xlabel("time [s]");
ylabel("angle [deg]");
%ylim([-0.5 0.5]);
grid on;
hold off;
legend({"r_{measured} [deg]","r_{estimated} [deg]", "r [deg]"}, "Location", "northeast");



% %% 2.f ------------------------------------------------------------
% 
% % Augmented parameters
% W_chi = 8; % was 15
% omega_n_chi = 1/W_chi * omega_n_phi;
% 
% % Course control parameters
% k_p_chi = 2 * zeta_chi * omega_n_chi * V_g/g;     
% k_i_chi = omega_n_chi^2 * V_g/g;  
% 
% out2 = sim('A2_Part3_simulink_model', sim_time);
% 
% figure('rend','painters','pos',[10 10 750 400])
% hold on;
% plot(out2.chi, "b");
% plot(out2.chi_control, "r");
% plot(out2.delta_control, "m--")
% xlim([90 200])
% title("Task 2f, controller performance, full model without anti-windup");
% xlabel("time [s]");
% ylabel("angle [deg]");
% grid on;
% hold off;
% legend({"chi [deg]", "chi_c [deg]", "delta_a [deg]"}, "Location", "northeast");
% 
% figure('rend','painters','pos',[10 10 750 400])
% hold on;
% plot(out2.e_chi_integral, "b");
% plot(out2.delta_control, "r");
% plot(out2.delta_saturated, "m--")
% xlim([90 200])
% title("Task 2f, wind up analysis full model without anti-windup");
% xlabel("time [s]");
% ylabel("angle [deg]");
% grid on;
% hold off;
% legend({"e chi integral [deg*s]", "delta [deg]", "delta saturated [deg]"}, "Location", "northeast");