% ROB 311, Fall 2022
% Single plane ball-bot
%
% Prof. Rouse & Senthur Raj
% University of Michigan
% 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear
clc
close all

% BALL-BOT PARAMETERS
m_a = 1.7194;                                                           % Mass of the body (kg)                                
m_k = 0.62;                                                             % Mass of the ball (kg)
m_w = 0.29;                                                             % Mass of the wheel (kg)
r_k = 0.121;                                                            % Radius of the ball (m)
r_w = 0.096;                                                            % Radius of the wheel (m)
L = 0.2311;                                                             % Distance to the center of mass of the body
I_k = 0.0036;                                                           % Moment of inertia of the ball (kgm^2)
I_w = 0.0015;                                                           % Moment of inertia of the wheel (kgm^2)
I_x = 0.0115;                                                           % Moment of inertia of the body in X-Z plane
I_y = I_x;                                                              % Moment of inertia of the body in the Y-Z plane
I_z = 0.0170;                                                           % Moment of inertia of the body in the X-Y plane
g = 9.81;                                                               % Gravity (m/s/s)

% MOTOR PARAMETERS - found from: https://www.pololu.com/product/4754/specs
N = 70;                                                                 % Transmission ratio
eta = 0.8;                                                              % Transmission efficiency
stall_torque = 2.65/70;                                                 % Motor properties obtained from Pololu website
stall_current = 5.5;
stall_voltage = 12;

noload_speed = 15.71*70;
noload_current = 0.2;

Kt = stall_torque/stall_current;                                        % Torque / back-EMF constant (Nm/A)
R = stall_voltage/stall_current;                                        % Terminal resistance (Ohms)
b = (noload_current*Kt)/noload_speed;                                   % Viscous damping (Nms/rad)
L = 0.023;                                                              % Motor inductance (H)
J = 0.000025;                                                           % Motor's rotor inertia (kgm^2) - estimated from 35mm Maxon motor

% VELOCITY PROFILE GENERATION
x_k_dot_final = 1;                                                      % Final ball-bot velocity (m/s)
t_final = 10;                                                           % Duration of the simulation (s)
t_constant = 6; % Ramp lasts from 2.5 s to t_constant                                                       % Time to reach constant final velocity (s)
dt = 1/200;                                                             % Simulation timestep
t_length = t_final/dt;                                                  % Length of time vector
t = linspace(0,t_final, t_length);                                      % Time vector (s)

x_k_dot = [zeros(1,500) linspace(0,x_k_dot_final, (round(t_length*(t_constant/t_final))-500))];% Velocity vector (m/s)
x_k_dot(length(x_k_dot):t_length) = x_k_dot_final;                      % Velocity vector (m/s)

% PLOT BALL-BOT TRAJECTORY (x_k)
figure
plot(t, x_k_dot, 'linewidth',2)
hold on
xlabel('Time (s)')
ylabel('Linear Velocity (m/s)')

% SMOOTH STEP FUNCTION AND PLOT TO SHOW DIFFERENCE
x_k_dot = smoothdata(x_k_dot, 'gaussian', 500);                         % Low pass filter to smooth disontinuous velocity profile
%x_k_dot = lowpass(x_k_dot,.1,1/dt,'ImpulseResponse','iir','Steepness',0.75); % This is a different way to smooth the signal--it uses a low-pass filter.
plot(t,x_k_dot, 'linewidth',2)

% POSITION AND ACCELERATION GENERATION
x_k_ddot = ddt(x_k_dot, dt);                                            % Ball-bot acceleration vector (m/s/s)
x_k = cumsum(x_k_dot)*dt;                                               % Integrating, ball-bot position vector (m)

% PLOTTING ACCELERATION
figure
plot(t, x_k_ddot,'linewidth',2)
xlabel('Time (s)')
ylabel('Linear Acceleration (m/s^2)')

% LEAN ANGLE - Some contraints on this variable to make the simulation
% realistic.  
theta_scale = .25;                                                      % Scaling factor for calcuation of lean angle
theta = theta_scale * x_k_ddot;                                         % Body lean angle (rad)                                                        % Lean angle starts at zero rads

% PLOTTING LEAN ANGLE
figure
plot(t, theta, 'linewidth',2)
xlabel('Time (s)')
ylabel('Lean Angle (rad)')

theta_dot = ddt(theta, dt);                                             % Angular velocty of the lean angle (rad/s)
theta_ddot = ddt(theta_dot, dt);                                        % Angular acceleration of the lean angle (rad/s/s)

% BALL ACCELERATION
phi_ddot = x_k_ddot / r_k;                                              % Angular acceleration of the ball
phi_dot = x_k_dot / r_k;                                                % Angular velocity of the ball

% TEMP VARIABLE
gamma = L * m_a + (r_k + r_w) * m_w;                                    % Temporary variable gamma

% CONTACT FORCE
F_w_2 = (m_a + m_w) * (g * sin(theta) - r_k * phi_ddot .* cos(theta)) - gamma .* theta_ddot;    % Contact force between wheel and ball

% WHEEL ACCELERATION
psi_ddot = (r_k / r_w) * (phi_ddot - theta_ddot) - theta_ddot;          % Angular acceleration of the wheels (rad/s/s)
psi_dot = (r_k / r_w) * (phi_dot - theta_dot) - theta_dot;              % Angular velocity of the wheels (rad/s)

% REQUIRED WHEEL TORQUE
T_x = I_w * psi_ddot + F_w_2 * r_w;                                     % Total wheel torque (Nm)

% MOTOR CURRENT AND VOLTAGE
i_winding = (T_x/(N * eta) + J * psi_ddot * N + b * psi_dot * N)/Kt;    % Motor current (A)
di_winding = ddt(i_winding, dt);                                        % Derivative of motor current
v_winding = i_winding * R + (Kt * psi_dot * N) + L * di_winding;        % Motor voltage (V)

% MORE PLOTTING 
figure
plot(t, T_x, 'linewidth', 2)
xlabel('Time (s)')
ylabel('Torque (Nm)')

figure;
subplot(211)
plot(t, i_winding, 'linewidth', 2)
xlabel('Time (s)')
ylabel('Motor Current (A)')
subplot(212)
plot(t, v_winding, 'linewidth', 2)
xlabel('Time (s)')
ylabel('Motor Voltage (V)')
