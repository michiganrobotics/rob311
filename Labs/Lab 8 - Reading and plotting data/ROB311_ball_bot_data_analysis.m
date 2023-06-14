% ROB 311, Fall 2022
% Data importing / plotting from ball-bot
%
% University of Michigan
% 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear
clc
close all

alpha = pi/4;                                                               % Wheel contact angle--for our design this is 45 deg
beta = pi/2;                                                                % This is the alignment of the X-Y axes with the motor triad--for us this is 90 deg (to align M1 with the y-axis)
Rk = 0.11925;                                                               % Basketball radius
Rw = 0.04778;                                                               % Wheel radius

% Input file name -  this will have to updated with the name of the trial
% you are analyzing
filename = 'ROB311_Test1.txt';

% Use file name to load / create data matrix
eval(['load ' filename ';']);                                               % Eval commands execute a text string as a command
eval(['data = ' erase(filename,'.txt') ';']);

% Define variables from data - these correspond to the order described in
% the read sensor demo
index = data(:,1);                                                          % Loop index (unitless)
time = data(:,2);                                                           % Time (s)
theta_x = data(:,3);                                                        % Rotation around the x-axis
theta_y = data(:,4);                                                        % Rotation around the y-axis

% Plotting
figure
hold on
plot(time, theta_x, 'linewidth',2)
plot(time, theta_y, 'linewidth',2)
legend('\theta_x', '\theta_y')
xlabel('Time (s)')
ylabel('Angle (rad)')