clc; clear;
close all;
addpath('functions');
%% UR5e dh parameters
a = [0, -0.425, -0.3922, 0, 0, 0];
d = [0.1625, 0, 0, 0.1333, 0.0997, 0.0996];
alpha = [pi/2, 0, 0, pi/2, -pi/2, 0];
% zero position
theta0 = [0 0 0 0 0 0]*pi/180;
% visualization
ur5eDemo3D(theta0, 40, 0, 0, 1, 0);
Endeffector_f = [];
Endeffector_l = [];
Endeffector_o = [];
Endeffector_w = [];
%% motion
% each letter's time
T = 20; % s
% waypoint
F = [-175 100 0
     -275 100 0
     -275 -100 0
     -275 20 0
     -175 20 0];
L = [-125 100 0
     -125 -100 0
     -25 -100 0];
W = [150 100 0
     190 -100 0
     230 100 0
     270 -100 0
     310 100 0];
%% IK 1
% R = eulerR(-pi/2, 0, 0);
R = [0 0 -1; 0 -1 0; -1 0 -0];
p = [-500, -150, 500]'*0.001;
H = [R p; 0 0 0 1];
theta_sol = ikineUR5e(H, d, a);
% choose the optimal angle
theta_opt = optTheta(theta0, theta_sol);
% trajectory
qf = [];
for m = 1: length(theta_opt)
    q_array=[0, theta_opt(m)*180/pi];
    t_array=[0, 10];
    v_array=[0, 0];
    a_array=[0, 0];
    [t_trajf, q_traj, v_traj, a_traj] = quintic(t_array, q_array, v_array, a_array, 0.2);
    qf = [qf q_traj'];
end
% p2p
q1 = qf*pi/180;
Endeffector = [];
for t = 1: length(t_trajf)
    % end effector trajectory
    [~, JointPos0] = fkineUR5e(q1(t, :), d, a, alpha);
%     Endeffector = [Endeffector; JointPos0(end, :)];
    % after optimization
    ur5eDemo3D(q1(t, :), 40, Endeffector*1000, 0, 1, t_trajf(t));
end
% start pause
for pause_t = 0: 0.5: 2
    % end effector trajectory
    [~, JointPos0] = fkineUR5e(q1(t, :), d, a, alpha);
%     Endeffector = [Endeffector; JointPos0(end, :)];
    % after optimization
    ur5eDemo3D(q1(t, :), 40, Endeffector*1000, 0, 1, pause_t);
end
%% F
[tp_f, vp] = lettertrajectory(F, 21, 0.5);
tp_f = tp_f(1:end-1);
% Endeffector = [];
q2 = qf(end, :)*pi/180;
num = 1;
for t = tp_f
    v = vp(:, num)*0.001; % m/s
    num = num + 1;
    % FK
    [T, JointPos0] = fkineUR5e(q2, d, a, alpha);
    % end effector trajectory
    Endeffector_f = [Endeffector_f; JointPos0(end, :)];
    Endeffector = Endeffector_f;
    [~, JointPos] = ur5eDemo3D(q2, 40, Endeffector*1000, 1, 1, t);
    % Inverse Jacobian
    J = mJacobian(T);
    dq = J'*(J*J')^(-1)*v;
    % trajectory
    q2p = q2;
    q2 = q2 + dq';
end
% end pause
for pause_t = 0: 0.5: 2
    % end effector trajectory
    [~, JointPos0] = fkineUR5e(q1(t, :), d, a, alpha);
%     Endeffector = [Endeffector; JointPos0(end, :)];
    % after optimization
    ur5eDemo3D(q2p, 40, Endeffector*1000, 1, 1, pause_t);
end
%% IK 2
% R = eulerR(-pi/2, 0, 0);
R = [0 0 -1; 0 -1 0; -1 0 -0];
p = [-500, -100, 500]'*0.001;
H = [R p; 0 0 0 1];
theta_sol = ikineUR5e(H, d, a);
% choose the optimal angle
theta_opt = optTheta(q2p, theta_sol);
% trajectory
ql = [];
for m = 1: length(theta_opt)
    q_array=[q2p(m)*180/pi, theta_opt(m)*180/pi];
    t_array=[0, 5];
    v_array=[0, 0];
    a_array=[0, 0];
    [t_traj, q_traj, v_traj, a_traj] = quintic(t_array, q_array, v_array, a_array, 0.2);
    ql = [ql q_traj'];
end
% p2p
q1 = ql*pi/180;
% Endeffector = [];
for t = 1: length(t_traj)
    % end effector trajectory
    [~, JointPos0] = fkineUR5e(q1(t, :), d, a, alpha);
%     Endeffector = [Endeffector; JointPos0(end, :)];
    % after optimization
    ur5eDemo3D(q1(t, :), 40, Endeffector*1000, 1, 1, t_traj(t));
end
% start pause
for pause_t = 0: 0.5: 2
    % end effector trajectory
    [~, JointPos0] = fkineUR5e(q1(t, :), d, a, alpha);
%     Endeffector = [Endeffector; JointPos0(end, :)];
    % after optimization
    ur5eDemo3D(q1(t, :), 40, Endeffector*1000, 1, 1, pause_t);
end
%% L
[tp_l, vp] = lettertrajectory(L, 20.5, 0.5);
% Endeffector = [];
q2 = ql(end, :)*pi/180;
num = 1;
for t = tp_l
    v = vp(:, num)*0.001; % m/s
    num = num + 1;
    % FK
    [T, JointPos0] = fkineUR5e(q2, d, a, alpha);
    % end effector trajectory
    Endeffector_l = [Endeffector_l; JointPos0(end, :)];
    Endeffector = [Endeffector_f; Endeffector_l];
    [~, JointPos] = ur5eDemo3D(q2, 40, Endeffector*1000, 1, 1, t);
    % Inverse Jacobian
    J = mJacobian(T);
    dq = J'*(J*J')^(-1)*v;
    % trajectory
    q2p = q2;
    q2 = q2 + dq';
end
% end pause
for pause_t = 0: 0.5: 2
    % end effector trajectory
    [~, JointPos0] = fkineUR5e(q1(t, :), d, a, alpha);
%     Endeffector = [Endeffector; JointPos0(end, :)];
    % after optimization
    ur5eDemo3D(q2p, 40, Endeffector*1000, 1, 1, pause_t);
end
%% IK 3
% R = eulerR(-pi/2, 0, 0);
R = [0 0 -1; 0 -1 0; -1 0 -0];
p = [-500, 125, 400]'*0.001;
H = [R p; 0 0 0 1];
theta_sol = ikineUR5e(H, d, a);
% choose the optimal angle
theta_opt = optTheta(q2p, theta_sol);
% trajectory
qo = [];
for m = 1: length(theta_opt)
    q_array=[q2p(m)*180/pi, theta_opt(m)*180/pi];
    t_array=[0, 5];
    v_array=[0, 0];
    a_array=[0, 0];
    [t_traj, q_traj, v_traj, a_traj] = quintic(t_array, q_array, v_array, a_array, 0.2);
    qo = [qo q_traj'];
end
% p2p
q1 = qo*pi/180;
% Endeffector = [];
for t = 1: length(t_traj)
    % end effector trajectory
    [~, JointPos0] = fkineUR5e(q1(t, :), d, a, alpha);
%     Endeffector = [Endeffector; JointPos0(end, :)];
    % after optimization
    ur5eDemo3D(q1(t, :), 40, Endeffector*1000, 1, 1, t_traj(t));
end
% start pause
for pause_t = 0: 0.5: 2
    % end effector trajectory
    [~, JointPos0] = fkineUR5e(q1(t, :), d, a, alpha);
%     Endeffector = [Endeffector; JointPos0(end, :)];
    % after optimization
    ur5eDemo3D(q1(t, :), 40, Endeffector*1000, 1, 1, pause_t);
end
%% O
ftt = 2*pi + 10*pi/180;
tt = 0: ftt/10: ftt;
ox = 75 + 50*cos(tt);
oy = 100*sin(tt);
oz = zeros(1, size(ox, 2));
O = [ox' oy' oz'];
[tp_o, vp] = lettertrajectory(O, 23, 0.5);
% Endeffector = [];
q2 = qo(end, :)*pi/180;
num = 1;
for t = tp_o
    v = vp(:, num)*0.001; % m/s
    num = num + 1;
    % FK
    [T, JointPos0] = fkineUR5e(q2, d, a, alpha);
    % end effector trajectory
    Endeffector_o = [Endeffector_o; JointPos0(end, :)];
    Endeffector = [Endeffector_f; Endeffector_l; Endeffector_o];
    [~, JointPos] = ur5eDemo3D(q2, 40, Endeffector*1000, 1, 1, t);
    % Inverse Jacobian
    J = mJacobian(T);
    dq = J'*(J*J')^(-1)*v;
    % trajectory
    q2p = q2;
    q2 = q2 + dq';
end
% end pause
for pause_t = 0: 0.5: 2
    % end effector trajectory
    [~, JointPos0] = fkineUR5e(q1(t, :), d, a, alpha);
%     Endeffector = [Endeffector; JointPos0(end, :)];
    % after optimization
    ur5eDemo3D(q2p, 40, Endeffector*1000, 1, 1, pause_t);
end
%% IK 4
% R = eulerR(-pi/2, 0, 0);
R = [0 0 -1; 0 -1 0; -1 0 -0];
p = [-500, 175, 500]'*0.001;
H = [R p; 0 0 0 1];
theta_sol = ikineUR5e(H, d, a);
% choose the optimal angle
theta_opt = optTheta(q2p, theta_sol);
% trajectory
qw = [];
for m = 1: length(theta_opt)
    q_array=[q2p(m)*180/pi, theta_opt(m)*180/pi];
    t_array=[0, 5];
    v_array=[0, 0];
    a_array=[0, 0];
    [t_traj, q_traj, v_traj, a_traj] = quintic(t_array, q_array, v_array, a_array, 0.2);
    qw = [qw q_traj'];
end
% p2p
q1 = qw*pi/180;
% Endeffector = [];
for t = 1: length(t_traj)
    % end effector trajectory
    [~, JointPos0] = fkineUR5e(q1(t, :), d, a, alpha);
%     Endeffector = [Endeffector; JointPos0(end, :)];
    % after optimization
    ur5eDemo3D(q1(t, :), 40, Endeffector*1000, 1, 1, t_traj(t));
end
% start pause
for pause_t = 0: 0.5: 2
    % end effector trajectory
    [~, JointPos0] = fkineUR5e(q1(t, :), d, a, alpha);
%     Endeffector = [Endeffector; JointPos0(end, :)];
    % after optimization
    ur5eDemo3D(q1(t, :), 40, Endeffector*1000, 1, 1, pause_t);
end
%% W
[tp_w, vp] = lettertrajectory(W, 20, 0.5);
% Endeffector = [];
q2 = qw(end, :)*pi/180;
num = 1;
for t = tp_w
    v = vp(:, num)*0.001; % m/s
    num = num + 1;
    % FK
    [T, JointPos0] = fkineUR5e(q2, d, a, alpha);
    % end effector trajectory
    Endeffector_w = [Endeffector_w; JointPos0(end, :)];
    Endeffector = [Endeffector_f; Endeffector_l; Endeffector_o; Endeffector_w];
    [~, JointPos] = ur5eDemo3D(q2, 40, Endeffector*1000, 1, 1, t);
    % Inverse Jacobian
    J = mJacobian(T);
    dq = J'*(J*J')^(-1)*v;
    % trajectory
    q2p = q2;
    q2 = q2 + dq';
end
% end pause
for pause_t = 0: 0.5: 2
    % end effector trajectory
    [~, JointPos0] = fkineUR5e(q1(t, :), d, a, alpha);
%     Endeffector = [Endeffector; JointPos0(end, :)];
    % after optimization
    ur5eDemo3D(q2p, 40, Endeffector*1000, 1, 1, pause_t);
end
%% back
Back = [335, 100, 0
        335, 100, 100];
[tp, vp] = lettertrajectory(Back, 5, 0.5);
% Endeffector = [];
q2 = q2p;
num = 1;
for t = tp
    v = vp(:, num)*0.001; % m/s
    num = num + 1;
    % FK
    [T, JointPos0] = fkineUR5e(q2, d, a, alpha);
    % end effector trajectory
%     Endeffector = [Endeffector; JointPos0(end, :)];
    [~, JointPos] = ur5eDemo3D(q2, 40, Endeffector*1000, 1, 1, t);
    % Inverse Jacobian
    J = mJacobian(T);
    dq = J'*(J*J')^(-1)*v;
    % trajectory
    q2p = q2;
    q2 = q2 + dq';
end
%% Endeffector
plotLetterPos(Endeffector_f, tp_f, 'F');
plotLetterPos(Endeffector_l, tp_l, 'L');
plotLetterPos(Endeffector_o, tp_o, 'O');
plotLetterPos(Endeffector_w, tp_w, 'W');
