% mode selection
dc_mode  = DC_MODE.ZERO; % zero mode
nc_mode  = NC_MODE.CONSTANT; %constant mode
ref_mode = REF_MODE.MANUAL_REF;

% shaft input step config
nc_step_time  = 0;
nc_step_start = 0;
nc_step_final = 0;

% heading reference sine config
psi_d_amp  = 0;
psi_d_bias = 0;
psi_d_freq = 0;

% surge reference step config
u_d_step_time   = 0;
u_d_step_start  = 0;
u_d_step_final  = 0;

% heading PID params
K_p_psi = 0;
K_i_psi = 0;
K_d_psi = 0;

% surge PPC params
k_reg = 0;
r_reg = 0;