% ROB 311, Fall 2022
% Torque conversion demonstration
% X-Y planar torque analysis
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
rw = 0.04778;                                                               % Wheel radius


% Please fill in the equations below--if you use the naming / vector sizes
% described below, the quiver plot will show you the locations of the
% contact forces that produce the torques, shown at their real and virtual
% locations.

%% TORQUE CONVERSION 

Tx =                                                                        % X-axis torque
Ty =                                                                        % Y-axis torque
    
Fx =                                                                        % Forces on virtual wheels
Fy =                                                                        % Forces on virtual wheels

T1 =                                                                        % Wheel 1 torque
T2 =                                                                        % Wheel 2 torque
T3 =                                                                        % Wheel 3 torque

%% PLANAR FORCES

Fw1 =                                                                       % Tangential Wheel 1 force vectors (1 x 2 vector)
Fw2 =                                                                       % Tangential Wheel 2 force vectors (1 x 2 vector)
Fw3 =                                                                       % Tangential Wheel 3 force vectors (1 x 2 vector)

Fws = [Fw1; Fw2; Fw3];

%% PLANAR CONTACT POINTS

rw1 =                                                                       % Location of Wheel 1 (1 x 2 vector)
rw2 =                                                                       % Location of Wheel 2 (1 x 2 vector)
rw3 =                                                                       % Location of Wheel 3 (1 x 2 vector)

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
quiver([rws(:,1);0], [rws(:,2);0], [Fws(:,1);Fx], [Fws(:,2);Fy], 'linewidth', 2) % Plotting real wheel force vectors
xlabel('X-axis (m)')
ylabel('Y-axis (m)')

