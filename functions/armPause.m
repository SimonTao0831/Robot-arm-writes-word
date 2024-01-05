function [t_p2p, q_p2p, v_p2p, a_p2p] = armPause(t_p2p, q_p2p, v_p2p, a_p2p, pause_t, eta)
t = eta: eta: pause_t; % pause time
q_pause = repmat(q_p2p(end, :), length(t), 1); % n x 6
v_pause = repmat(v_p2p(end, :), length(t), 1); % n x 6
a_pause = repmat(a_p2p(end, :), length(t), 1); % n x 6
t_p2p = [t_p2p, t_p2p(end) + t];
q_p2p = [q_p2p; q_pause];
v_p2p = [v_p2p; v_pause];
a_p2p = [a_p2p; a_pause];
end

