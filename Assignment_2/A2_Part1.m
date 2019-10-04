clear; close all;

%% Assignment 2 Part 1

A2_model;

%% Problem 1

V_g = V_a;
%% Problem 2

% 2.a)

a_phi_1 = -0.65;
a_phi_2 = 2.87;

H_p_delta_a = tf([0 a_phi_1], [1 a_phi_2]);

% 2.b)

delta_a_max = 30;
e_phi_max = 15;
zeta_phi = 0.707;
k_p_chi = 0; k_i_chi = 0;  
k_p_phi = 0; k_i_phi = 0;  k_d_phi = 0;
d = 1.5;
%2.d)
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



