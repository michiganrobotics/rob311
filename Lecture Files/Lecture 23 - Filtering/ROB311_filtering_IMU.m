%% ROB 311 Filtering Examlpe
% This example will load data, plot the FFT, and filter
%
% ROB 311 - Professor Rouse, Fall 2022

close all
clc
clear 

%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Load Data %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
load ROB311_Example_IMU_Data_Prosthesis.txt

data = ROB311_Example_IMU_Data_Prosthesis;
data = data(100:end-500,:);
time = (data(:,2)+data(:,17))./2;
brk = data(:,3);
thetak = data(:,4);
thetakdot = data(:,5);
vert_load = data(:,6);
accy = data(:,7);
accz = data(:,8);
gyrox = data(:,9);
motor_angle = data(:,11);
motor_current = data(:,12);
motor_voltage = data(:,13);
motor_power = motor_current.*motor_voltage;
controller_state = data(:,14);
KP = data(:,15);
KD = data(:,16);
dt = [0.01; time(2:end) - time(1:end-1)];

%%%%%%%%%%%%%%%%%%%%%%%%%%% FILTERING AND FFT %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%Filtering
cutoff = 3;
data_to_be_filtered = accz;
data_filtered = low_filt(1/mean(dt),2,cutoff,data_to_be_filtered);

%Frequency content
[amplitude, frequency] = FFT(1/mean(dt), data_to_be_filtered);
[amplitude_filtered, frequency_filtered] = FFT(1/mean(dt), low_filt(1/mean(dt),2,3,data_to_be_filtered));


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% PLOTTING %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% PID control
figure;
plot(time, thetak,'linewidth',2)
hold on
plot(time, motor_angle,'linewidth',2)
xlabel('Time (s)')
ylabel('Knee Angle (deg)')
legend('Knee Angle', 'Motor Angle')


figure
subplot(311)
plot(time, motor_voltage,'linewidth',2)
xlabel('Time (s)')
ylabel('Voltage (V)')
subplot(312)
plot(time, motor_current,'r','linewidth',2)
xlabel('Time (s)')
ylabel('Current (A)')
subplot(313)
plot(time, motor_power,'k','linewidth',2)
xlabel('Time (s)')
ylabel('Power (W)')


figure
plot(time, accz,'linewidth',2)
hold on
plot(time, data_filtered,'linewidth',2)
xlabel('Time (s)')
ylabel('Acceleration (g)')

figure
subplot(121)
plot(time, data_to_be_filtered,'linewidth',2)
hold on
plot(time, data_filtered,'linewidth',2)
xlabel('Time (s)')
ylabel('Acceleration (g)')                                              % Units will need to change if the signal being investigated gets changed
subplot(122)
plot(frequency, amplitude,'linewidth',2)
hold on
plot(frequency_filtered, amplitude_filtered,'linewidth',2)
xlabel('Frequency')
ylabel('Power (Db/Hz)')