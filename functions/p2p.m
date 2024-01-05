function [t_traj, qf, vf, af] = p2p(theta_start, theta_end, tf, eta)
% trajectory
qf = [];
vf = [];
af = [];
% calculate one joint
for m = 1: length(theta_end)
    q_array=[theta_start(m), theta_end(m)];
    t_array=[0, tf];
    v_array=[0, 0];
    a_array=[0, 0];
    [t_traj, q_traj, v_traj, a_traj] = quintic(t_array, q_array, v_array, a_array, eta);
    qf = [qf q_traj'];
    vf = [vf v_traj'];
    af = [af a_traj'];
end
end
