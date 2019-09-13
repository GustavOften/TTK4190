clear;

% parameters
K = 0.1;
U = 5;
b = 0.001;
T = 20;        
rudder_sat = 20*pi/180;
   
% Control parameters
k_p = 5e-5;
k_i = 5e-8; 
k_d = 2e-2;

% Sim parameters
h = 0.05;
sim_time = 5000; 
N = sim_time/h;
time_vec    = (0:h:sim_time);
time_vec_2  = (0:h:sim_time + h); %for longer state vectors
 
% init states
x_0 = 0;
y_0 = 100;
psi_0 = 0;
r_0 = 0;

% init state vectors
x = zeros(1, N);   
y = zeros(1, N);   
y_dot = zeros(1, N);
y_integral = 0;
    
x(1) = x_0;
y(1) = y_0;

psi = zeros(1, N); 
r = zeros(1, N); 

delta = zeros(1, N);

psi(1) = psi_0;
r(1) = r_0;

%Simulation
    
    for i = 1:N+1
        t = (i-1)*h;
        
        y_dot(i)    = U*psi(i);
        y_integral  = y_integral + y(i)*h; 
        
        delta(i) = -k_p*y(i) - k_i*y_integral- k_d*y_dot(i); %control input update
        if delta(i) >= rudder_sat
            delta(i) = rudder_sat;
        elseif delta(i) <= -rudder_sat
            delta(i) = -rudder_sat;
        end
        x(i+1)      = x(i)      + U*cos(psi(i))*h;
        y(i+1)      = y(i)      + U*sin(psi(i))*h;
        psi(i+1)    = psi(i)    + r(i)*h;
        r(i+1)      = r(i)      - (h/T)*r(i) + (h/T)*(K*delta(i)+b);
       
    end
%% Plot

figure (1);
hold on;
plot(x, y, 'b');
hold off;
grid on;
title('Position');
xlabel('North [m]'); 
ylabel('East [m]');

figure (2);
hold on;
plot(time_vec, delta, 'b');
hold off;
grid on;
title('Control input');
xlabel('Time [s]'); 
ylabel('Rudder angle [rad]');
xlim([0, 4500])

figure (3);
hold on
plot(time_vec_2, psi, 'b');
hold off;
grid on;
title('Heading');
xlabel('Time [s]');
ylabel('Yaw angle [rad]');
xlim([0, 4500])

figure (4);
hold on
plot(time_vec_2, r, 'b');
hold off;
grid on;
title('Heading rate');
xlabel('Time [s]');
ylabel('Yaw angle rate [rad/s]');
xlim([0, 4500]);