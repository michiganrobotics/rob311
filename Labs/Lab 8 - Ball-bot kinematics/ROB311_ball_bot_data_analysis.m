% ROB 311, Fall 2022
% Data importing / plotting from ball-bot
% Kinematic analysis of an upside down ball-bot
%
% Prof. Elliott Rouse
% University of Michigan
% 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear
clc
close all                                                                   % If you want to compare different trials, you may wish to comment this out

alpha = pi/4;                                                               % Wheel contact angle--for our design this is 45 deg
beta = pi/2;                                                                % This is the alignment of the X-Y axes with the motor triad--for us this is 90 deg (to align M1 with the y-axis)
Rk = 0.11925;                                                               % Basketball radius
Rw = 0.04778;                                                               % Wheel radius

% Input file name -  this will have to updated with the name of the trial
% you are analyzing
filename = 'ROB311_Test8.txt';

% Use file name to load / create data matrix
eval(['load ' filename ';']);                                               % Eval commands execute a text string as a command
eval(['data = ' erase(filename,'.txt') ';']);

% Define variables from data - these correspond to the order described in
% the read sensor demo
index = data(:,1);                                                          % Loop index (unitless)
time = data(:,2);                                                           % Time (s)
theta_x = data(:,3);                                                        % Rotation around the x-axis (rad)
theta_y = data(:,4);                                                        % Rotation around the y-axis (rad)
T1 = data(:,5);                                                             % Motor 1 torque (Nm)
T2 = data(:,6);                                                             % Motor 2 torque (Nm)
T3 = data(:,7);                                                             % Motor 3 torque (Nm)
phi_x = data(:,8);                                                          % Ball rotation around the x-axis (rad)
phi_y = data(:,9);                                                          % Ball rotation around the y-axis (rad)
phi_z = data(:,10);                                                         % Ball rotation around the z-axis (rad)
psi_1 = data(:,11);                                                         % Wheel 1 rotation (rad)
psi_2 = data(:,12);                                                         % Wheel 2 rotation (rad)
psi_3 = data(:,13);                                                         % Wheel 3 rotation (rad)

dt = mean(diff(time));                                                      % Time step for integration
x_k = Rk*cumsum(phi_y - phi_y(1))*dt;                                       % Integrate Rk*phi to obtain translation in X-Y plane
y_k = Rk*cumsum(phi_x-phi_x(1))*dt;                                         % Integrate Rk*phi to obtain translation in X-Y plane

% Plotting
figure
hold on
plot(time, theta_x, 'linewidth',2)
plot(time, theta_y, 'linewidth',2)
legend('\theta_x', '\theta_y')
xlabel('Time (s)')
ylabel('Angle (rad)')

figure
hold on
subplot(311)
plot(time, phi_x, 'linewidth',2)
ylabel('Rot about x-ax (r)')
subplot(312)
plot(time, phi_y, 'linewidth',2)
ylabel('Rot about y-ax (r)')
subplot(313)
plot(time, phi_z, 'linewidth',2)
ylabel('Rot about z-ax (r)')
xlabel('Time (s)')

figure
hold on
subplot(311)
plot(time, psi_1, 'color',[.6 .6 .6], 'linewidth',2)
ylabel('Rot of M1 (r)')
subplot(312)
plot(time, psi_2, 'color',[.6 .6 .6],  'linewidth',2)
ylabel('Rot of M2 (r)')
subplot(313)
plot(time, psi_3, 'color',[.6 .6 .6],  'linewidth',2)
ylabel('Rot of M3 (r)')
xlabel('Time (s)')

figure
plot(x_k,y_k,'k','linewidth',2)
axis equal
xlabel('X distance (m)')
ylabel('Y distance (m)')