% ROB 311, Fall 2022
% Torque conversion demonstration
%
% University of Michigan
% 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear
clc
close all

alpha = pi/4;                                                               % Wheel contact angle--for our design this is 45 deg
beta = pi/2;                                                                % This is the alignment of the X-Y axes with the motor triad--for us this is 90 deg (to align M1 with the y-axis)
Rk = 0.11925;
Rw = 0.04778; 

%% TORQUE CONVERSION

Tx = 1;                                                                     % X-axis torque
Ty = 10;                                                                     % Y-axis torque

Fx = Ty/Rw;                                                                 % Forces on virtual wheels
Fy = Tx/Rw;                                                                 % Forces on virtual wheels

T1 = (1/3)*((2/cos(alpha))*(Tx*cos(beta)-Ty*sin(beta)));                    % Wheel 1 torque
T2 = (1/3)*((1/cos(alpha))*(sin(beta)*(-sqrt(3)*Tx+Ty)-cos(beta)*(Tx+sqrt(3)*Ty))); % Wheel 2 torque
T3 = (1/3)*((1/cos(alpha))*(sin(beta)*(sqrt(3)*Tx+Ty)+cos(beta)*(-Tx+sqrt(3)*Ty))); % Wheel 3 torque

%% PLANAR FORCES

Fw1 = (T1/Rw)*[-sin(beta) cos(beta)];                                       % Tangential Wheel 1 forces
Fw2 = (T2/Rw)*[-sin(beta+(2*pi/3)) cos(beta+(2*pi/3))];                     % Tangential Wheel 2 forces
Fw3 = (T3/Rw)*[-sin(beta-(2*pi/3)) cos(beta-(2*pi/3))];                     % Tangential Wheel 3 forces

Fws = [Fw1; Fw2; Fw3];

%% PLANAR CONTACT POINTS

rw1 = Rk*[cos(beta)*sin(alpha), sin(beta)*sin(alpha)];                      % Location of Wheel 1
rw2 = Rk*[cos(beta + (2*pi/3))*sin(alpha), sin(beta + (2*pi/3))*sin(alpha)];% Location of Wheel 2
rw3 = Rk*[cos(beta - (2*pi/3))*sin(alpha), sin(beta - (2*pi/3))*sin(alpha)];% Location of Wheel 3

rws = [rw1; rw2; rw3];

%% SANITY CHECK
% Recompute original torques to verify
Tx_recalc = cos(alpha)*(T1*cos(beta)-T2*sin(beta+(pi/6))+T3*sin(beta-(pi/6)));
Ty_recalc = cos(alpha)*(-T1*sin(beta)-T2*cos(beta + (pi/6))+T3*cos(beta-(pi/6)));
T_error = Tx-Tx_recalc + Ty-Ty_recalc

%% QUIVER PLOTTING

figure
hold on
viscircles([0,0],Rk,'linewidth',2);                                         % Plotting ball 
viscircles([0,0],Rk*sin(alpha),'linewidth',2, 'color','k');                 % Plotting contact arc
quiver([rws(:,1);0], [rws(:,2);0], [Fws(:,1);Fx], [Fws(:,2);Fy], 'linewidth', 2)              % Plotting real wheel force vectors
xlabel('X-axis (m)')
ylabel('Y-axis (m)')

