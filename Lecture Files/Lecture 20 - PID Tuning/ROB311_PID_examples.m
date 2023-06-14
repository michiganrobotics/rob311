% ROB 311, Fall 2022
% PID Control example
%
% Professor Rouse
% University of Michigan
%
% This script walks through P, PI, and PID controllers with an example
% mechanical system (brushed motor).  For the in-class exercise, you will 
% change the plant dynamics to the commented set, then re-tune the PID 
% controller at the bottom of the script.
%
% When finished upload a plot of your response to Canvas 
% 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear
clc
close all

 

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  PLANT  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 
% This example describes systems using transfer functions / Laplace
% transforms (which are not covered in this class). So don't worry too much
% about the specific Matlab descriptions of the system; focus on the tuning
% of the controller gains.

% Equations of a DC motor 
% (Js^2+bs)*O = K*I
% (Ls + R)I+KsO = V
% input: Voltage
% output: Angle

J=0.1; % Rotor inertia
%J=0.3;                                                                     % Rotor inertia - uncomment for excercise
b=0.1;                                                                      % Viscous damping
%b=0;                                                                       % Viscous damping - uncomment for excercise
K=0.01;                                                                     % Motor torque contant
%K=0.35;                                                                    % Motor torque contant - uncomment for excercise
R=1;                                                                        % Winding resistance
%R=10;                                                                      % Winding resistance - uncomment for excercise
L=0.5;                                                                      % Inductance
%L=1;                                                                       % Inductance - uncomment for excercise
nump=K;  
denp=[(J*L) ((J*R)+(L*b)) ((b*R)+K^2)];
plant = tf(nump,denp);
figure
impulse(plant)                                                              % Plot impulse response of plant - this tells you a bit about what the system being controlled responds like
title('Impulse response of plant')

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% CONTROLLERS %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Proportional control

% P control - vector of three p-gain values to plot at once
kp = [25 75 200];                                                           % Proportional gain values to simulate / plot

% Controller simulation - build the closed loop controllers
numc1 = kp;
denc1 = 1;
[n1a,d1a]=cloop(conv(numc1(1),nump),denc1*denp);                            % Create closed loop system with controller and system T
[n1b,d1b]=cloop(conv(numc1(2),nump),denc1*denp);
[n1c,d1c]=cloop(conv(numc1(3),nump),denc1*denp);

t=0:0.001:5;                                                                % Time vector
                                                                            % Plotting
figure
hold on
step(n1a,d1a,t)                                                             % Plot step responses to see difference in behavior
step(n1b,d1b,t)                                                             % Plot step responses to see difference in behavior
step(n1c,d1c,t)                                                             % Plot step responses to see difference in behavior 
legend('Kp=25','Kp=75','Kp=200')                                            % Update if you change kp
title('Step response with Proportional Control')

%% Proportional-Intregral Control

% PI control
kp = 75;                                                                    % Keep at 75 for reference to the P controller
ki = 20;                                                                    % Change from zero to see the effect of the I term

% Controller simulation
numc2 = [kp ki];
denc2 = [1 0];
controller = tf(numc2, denc2);
[n2,d2]=cloop(conv(numc2,nump),conv(denc2,denp));

t=0:0.001:5; 
figure
hold on
step(n2,d2,t)
step(n1b,d1b,t)
legend('PI Controller', 'P Controller')
title('Step response with Proportinal-Integral Control')


%% Proportional-Integral-Derivative control
% Tune these gains for the ROB311 in-class exercise (uncomment changes to
% plant dynamics is cell 1)
% Once you're finished, uplaod a PDF of your response to Canvas
%
% You'll be graded by how close your response looks to the step function

% PID control
kp = 0;                                                                  % Tune these to see how it affects the output
ki = 0;
kd = 0;

% Controller simulation
numc3 = [kd kp ki];
denc3 = [1 0];
controller = tf(numc3, denc3);
[n3,d3]=cloop(conv(numc3,nump),conv(denc3,denp));

t=0:0.001:5;                                                                % Plotting
figure
step (n3,d3,t) 
title('Step response with Proportinal-Integral-Derivative Control')



