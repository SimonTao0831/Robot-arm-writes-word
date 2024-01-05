clc; clear;
close all;
addpath('functions');
%% Parameters
% UR5e dh parameters
a = [0, -0.425, -0.3922, 0, 0, 0];
d = [0.1625, 0, 0, 0.1333, 0.0997, 0.0996];
alpha = [pi/2, 0, 0, pi/2, -pi/2, 0];
% zero position
theta0 = [0 0 0 0 0 0]*pi/180; % [rad]
% trajectory parameters
goal_t = 60; % each letter's time [s], 60
% plane 1
R = eulerR(pi/2, 0, pi/2);
p = [-500, 0, 500]';
% plane 2
% R = eulerR(pi/2, 0, pi/4);
% p = [-300, 300, 500]';
interL = 10; % interpolation point distance [mm], 10
acct = 0.2; % accelerate time percentage [%], 0.2
dacct = 0.2; % decelerate time percentage [%], 0.2
accp = 0.2; % interpolation segments are used to accelerate [%], 0.2
daccp = 0.2; % interpolation segments are used to decelerate [%], 0.2
eta = 0.5; % sampling points [s], 0.5
v = 20; % expected velocity of motion between two points [mm/s], 20
pause_t = 2; % pause time [s], 2
%% 'FLOW' interpolation points
% F
F = [-175 100 0; -275 100 0; -275 -100 0; -275 20 0; -175 20 0];
% L
L = [-125 100 0; -125 -100 0; -25 -100 0];
% O
tt = -3*pi/2: 2*pi/10: pi/2;
ox = 75 + 50*cos(tt);
oy = 100*sin(tt);
oz = zeros(1, size(ox, 2));
O = [ox' oy' oz'];
% W
W = [150 100 0; 190 -100 0; 230 100 0; 270 -100 0; 310 100 0];
words{1} = F; words{2} = L; words{3} = O; words{4} = W; % put them in a cell array
%% Generate FLOW trajectroy
[total_t, total_q_traj, total_v_traj, total_a_traj, eva_t] = writeWords(words, R, p, theta0, v, interL, goal_t, acct, dacct, accp, daccp, eta, pause_t);
%% Video
Endeffector = [];
q = total_q_traj;
for t = 1: length(total_t)
    % end effector trajectory
    [~, JointPos0] = fkineUR5e(q(t, :), d, a, alpha);
    Endeffector = [Endeffector; JointPos0(end, :)];
    % after optimization
    ur5eDemo3D(q(t, :), 40, Endeffector*1000, 1, 1, roundn(total_t(t), 0));
end
%% Observe joint status
for joint_num = 1: 6
    jointPVA(joint_num, total_t, total_q_traj*180/pi, total_v_traj*180/pi, total_a_traj*180/pi, eva_t, goal_t, pause_t);
end
%% End effector status
EndeffectorPVA(Endeffector, total_t, eva_t, goal_t, pause_t);
