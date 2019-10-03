%% Assignment 2 Part 1

A2_model;

%% Problem 1



%% Problem 2

% 2.a)

a_phi_1 = -0.65;
a_phi_2 = 2.87;

H_p_delta_a = tf([0 a_phi_1], [1 a_phi_2]);

% 2.b)

delta_a_max = 30;
e_phi_max = 15;
zeta_phi = 0.707;