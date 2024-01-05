function [total_t, total_q_traj, total_v_traj, total_a_traj, eva_t] = writeWords(words, tf_R, tf_p, theta0, v, interL, goal_t, acct, dacct, accp, daccp, eta, pause_t)
%% UR5e dh parameters
a = [0, -0.425, -0.3922, 0, 0, 0];
d = [0.1625, 0, 0, 0.1333, 0.0997, 0.0996];
alpha = [pi/2, 0, 0, pi/2, -pi/2, 0];
total_t = [];
total_q_traj = [];
total_v_traj = [];
total_a_traj = [];
N = numel(words); % number of characters
R = tf_R*eulerR(pi, 0, -pi/2); % transfor plane to endeffector 
for n = 1: N
    %% words transformation
    tf_word = transfWord(words{n}, tf_R, tf_p);
    %% p2p
    p = tf_word(1, :)'*0.001;
    H = [R p; 0 0 0 1];
    theta_sol = ikineUR5e(H, d, a);
    % choose the start point
    if n == 1
        theta_start = theta0; % start from theta 0
    else
        theta_start = total_q_traj(end, :); % start from previous point
    end
    % choose the end point
    theta_end = optTheta(theta_start, theta_sol);
    % according to the distance, compute an approximate time
    eva_t(n) = evaTime(theta_start, theta_end, v);
    % generate trajectory
    [t_p2p, q_p2p, v_p2p, a_p2p] = p2p(theta_start, theta_end, eva_t(n), eta);
    % start pause
    [t_p2p, q_p2p, v_p2p, a_p2p] = armPause(t_p2p, q_p2p, v_p2p, a_p2p, pause_t, eta);
    %% write a character
    % generate trajectory
    [t_joint, q_traj, v_traj, a_traj] = p2pMultiLine(tf_word, interL, R, q_p2p(end, :), goal_t, acct, dacct, accp, daccp, eta);
    if n == 3 % when write O
        [t_joint, q_traj, v_traj, a_traj] = p2pCurve(tf_word, goal_t, R, theta0,  acct, dacct, accp, daccp, eta);
    end
    % end pause
    [t_joint, q_traj, v_traj, a_traj] = armPause(t_joint, q_traj, v_traj, a_traj, pause_t, eta);
    %% combine data of two processes
    segment_t = [t_p2p, t_joint + t_p2p(end)];
    segment_q_traj = [q_p2p; q_traj];
    segment_v_traj = [v_p2p; v_traj];
    segment_a_traj = [a_p2p; a_traj];
    %% final data
    if n == 1
        total_t = [total_t segment_t];
    else
        total_t = [total_t segment_t + total_t(end)];
    end
    total_q_traj = [total_q_traj; segment_q_traj];
    total_v_traj = [total_v_traj; segment_v_traj];
    total_a_traj = [total_a_traj; segment_a_traj];
end
end
