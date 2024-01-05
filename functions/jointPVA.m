function [] = jointPVA(joint, total_t, total_q_traj, total_v_traj, total_a_traj, eva_t, goal_t, pause_t)
cumeva_t = cumsum(eva_t);
f = figure();
f.Position(3:4) = [850 430];
letter_t = goal_t + pause_t*2;
subplot(3,1,1),plot(total_t(1: end), total_q_traj((1: end), joint)','r'),xlabel('t (s)'),ylabel('position (deg)');
for n = 1: length(eva_t)
    xline(cumeva_t(n) + letter_t*(n - 1),'--b');xline(cumeva_t(n) + letter_t*n,'--r');
end
subplot(3,1,2),plot(total_t(1: end), total_v_traj((1: end), joint)','b'),xlabel('t (s)'),ylabel('velocity (deg/s)');
for n = 1: length(eva_t)
    xline(cumeva_t(n) + letter_t*(n - 1),'--b');xline(cumeva_t(n) + letter_t*n,'--r');
end
subplot(3,1,3),plot(total_t(1: end), total_a_traj((1: end), joint)','g'),xlabel('t (s)'),ylabel('accelerate (deg/s^2)');
for n = 1: length(eva_t)
    xline(cumeva_t(n) + letter_t*(n - 1),'--b');xline(cumeva_t(n) + letter_t*n,'--r');
end
sgtitle(['Joint = ', num2str(joint)], 'FontSize', 10);

end

