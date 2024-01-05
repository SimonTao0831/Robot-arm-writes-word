function [eva_t] = evaTime(theta_start, theta_end, v)
% according to the distance, compute an approximate time
% v: desired velocity, mm/s
% UR5e dh parameters
a = [0, -0.425, -0.3922, 0, 0, 0];
d = [0.1625, 0, 0, 0.1333, 0.0997, 0.0996];
alpha = [pi/2, 0, 0, pi/2, -pi/2, 0];
[~, JointPos0] = fkineUR5e(theta_start, d, a, alpha);
[~, JointPos1] = fkineUR5e(theta_end, d, a, alpha);
start_P = JointPos0(end, :);
end_P = JointPos1(end, :);
dis = norm(end_P - start_P)*1000; % m -> mm
eva_t = round(dis/v);
end
