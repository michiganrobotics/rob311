%% Proportional-Integral-Derivative control
% 
% Use this mfile to complete HW5 Q2

close all
clc
clear 

%%%%%%%%%%%%%%%%%%%%%%%% DEFINITION OF PLANT %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
K = 1;
nump=K;                                                                     % Plant numerator
denp = [.2 1 5 10];                                                         % Plant denominator
plant = tf(nump,denp);

% IRF Plotting - if desired
% figure
% impulse(plant)                                                            % Plot impulse response of plant - this tells you a bit about what the system being controlled responds like
% title('Impulse response of plant')
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% CONTROL %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% PID control
kp = 0;                                                                  % Tune these to see how it affects the output
ki = 0;
kd = 0;

% Controller simulation
numc = [kd kp ki];
denc = [1 0];
cloop= feedback(tf(numc,denc)*tf(nump,denp),1);

% Step response for controller tuning
t = 0:.001:5;
step(cloop,t)


%%%%%%%%%%%%%%%%%%%%%%%%%%%% FREQUENCY RESPONSE TEST %%%%%%%%%%%%%%%%%%%%%%
% Setup input parameters
t = 0:0.001:200;                                                            % Duration of simulation 
f = 10;                                                                     % Frequency of sine wave
A = 5;                                                                      % Amplitude of sine wave
noise_level = 30;                                                           % Noise level

% Generating input and output response (don't edit this)
input = A*sin(2*pi*f*t);
input_noise = noise_level*rand(1,length(input));
input_wnoise = input + (input_noise-noise_level*.5);
output_wnoise = lsim(cloop,input_wnoise,t);
output = lsim(cloop,input,t);

%%%%%%%%%%%%%%%%%%%%%%%%%%%% PLOTTING %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

figure
plot(t,input,'linewidth',2)
hold on
plot(t,input_wnoise,'linewidth',2)
xlabel('Time (s)')
ylabel('Amplitude')
title('Input with added noise')

figure
plot(t,output,'linewidth',2)
hold on
plot(t,input,'linewidth',2)
xlabel('Time (s)')
ylabel('Output')
title('Input vs. Output')

figure
plot(t,output_wnoise,'linewidth',2)
hold on
plot(t,input,'linewidth',2)
xlabel('Time (s)')
ylabel('Output')
title('Input vs. Output w/noise')
