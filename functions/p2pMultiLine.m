function [total_t, total_q_traj, total_v_traj, total_a_traj] = p2pMultiLine(points, L, R, theta0, goal_t, acct, dacct, accp, daccp, eta)
% points: way points
% L: set the interpolation interval
% R: rotation matrix
% theta0: start point
% goal_t: total time
% acct: accelerate time
% dacct: decelerate time
% accp: interpolation segments are used to accelerate
% daccp: interpolation segments are used to decelerate
% eta: sampling points
%% UR5e dh parameters
a = [0, -0.425, -0.3922, 0, 0, 0];
d = [0.1625, 0, 0, 0.1333, 0.0997, 0.0996];
alpha = [pi/2, 0, 0, pi/2, -pi/2, 0];
total_t = [];
total_q_traj = [];
total_v_traj = [];
total_a_traj = [];
tf = allocationTime(points, goal_t);
%% generate the trajectory
% choose two interpolation points
for via = 1: size(points, 1) - 1
    %% position x, y, z
    % generate segments
    ave_points = averageDis(points(via, :), points(via + 1, :), L);
    %% inverse kinematics
    % get the theta of each point
    theta_opt = [];
    % how many segments
    for n = 1: size(ave_points, 1)
        % transforamtion matrix
        p = ave_points(n, :)'*0.001;
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
        [t_array, v_array, a_array] = autoVA(onejoint, 0, tf(via), acct, dacct, accp, daccp);
        q_array = theta_opt(:, m)';
        % quintic interpolation
        [t_joint, q_joint, v_joint, a_joint] = quintic(t_array, q_array, v_array, a_array, eta);
        q_traj = [q_traj q_joint'];
        v_traj = [v_traj v_joint'];
        a_traj = [a_traj a_joint'];
    end
    %% total trajectory
    if via == 1
        total_t = [total_t t_joint];
    else
        total_t = [total_t t_joint + total_t(end)];
    end
    total_q_traj = [total_q_traj; q_traj];
    total_v_traj = [total_v_traj; v_traj];
    total_a_traj = [total_a_traj; a_traj];
end
end
