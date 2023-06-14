% ROB 311, Fall 2022
% In-class example
%
% University of Michigan
% 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear
clc
close all

load ROB311_modeling_example.mat

l_a = 1;
m = 10;
J_load = m*l_a^2;

L = 
R = 
J = 
kt = 
N = 
b = 
g = 9.81;
ddtheta = ddt(dtheta,dt);

% CALCULATE TORQUE



% CALCULATE CURRENT



% CALCULATE VOLTAGE




% PLOTTING - below uses my variable naming convention; you will need to
% update the variable names if you use a different convention

figure; 
subplot(121)
plot(t,v_w,'linewidth',2)
xlabel('Time (s)')
ylabel('Voltage (V)')
subplot(122)
plot(t,i_w,'linewidth',2)
xlabel('Time (s)')
ylabel('Current (A)')
figure
subplot(121)
plot(t,rad2deg(theta), 'linewidth',2)
xlabel('Time (s)')
ylabel('Theta (^o)')
subplot(122)
plot(t,tau,'linewidth',2)
xlabel('Time (s)')
ylabel('Torque (Nm')



