function [t_joint, q_traj, v_traj, a_traj] = p2pCurve(tf_word, goal_t, R, theta0,  acct, dacct, accp, daccp, eta)
%% UR5e dh parameters
a = [0, -0.425, -0.3922, 0, 0, 0];
d = [0.1625, 0, 0, 0.1333, 0.0997, 0.0996];
alpha = [pi/2, 0, 0, pi/2, -pi/2, 0];
%% get the theta of each point
theta_opt = [];
% how many segments
for n = 1: size(tf_word, 1)
    % transforamtion matrix
    p = tf_word(n, :)'*0.001;
    T = [R p; 0 0 0 1];
    % IK
    theta_sol = ikineUR5e(T, d, a);
    % choose the optimal angle
    if n == 1 % first point, compare to the start point
        theta_opt = [theta_opt; optTheta(theta0, theta_sol)];
    else % the others are compared with the previous calculation of theta_opt
        theta_opt = [theta_opt; optTheta(theta_opt(end, :), theta_sol)];
    end
end
%% trajectory
q_traj = [];
v_traj = [];
a_traj = [];
for m = 1: size(theta_opt, 2)
    % choose one joint, get the theta points
    onejoint = theta_opt(:, m);
    % automatically generate the time points, velocity points and accelearte points
    [t_array, v_array, a_array] = autoVA(onejoint, 0, goal_t, acct, dacct, accp, daccp);
    q_array = theta_opt(:, m)';
    % quintic interpolation
    [t_joint, q_joint, v_joint, a_joint] = quintic(t_array, q_array, v_array, a_array, eta);
    q_traj = [q_traj q_joint'];
    v_traj = [v_traj v_joint'];
    a_traj = [a_traj a_joint'];
end
end
