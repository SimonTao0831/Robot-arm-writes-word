function [] = EndeffectorPVA(Endeffector, total_t, eva_t, goal_t, pause_t)
%% data
% position, mm
Endp_x = Endeffector(:, 1)*1000;
Endp_y = Endeffector(:, 2)*1000;
Endp_z = Endeffector(:, 3)*1000;
% velocity
Endv_x = [0; diff(Endp_x)];
Endv_y = [0; diff(Endp_y)];
Endv_z = [0; diff(Endp_z)];
% acceleration
Enda_x = [0; diff(Endv_x)];
Enda_y = [0; diff(Endv_y)];
Enda_z = [0; diff(Endv_z)];
% process time
cumeva_t = cumsum(eva_t);
% 60s to writer letter and 4s pause
letter_t = goal_t + 2*pause_t;
%% position
f1 = figure;
f1.Position(3:4) = [850 420];
sgtitle('The position of the end-effector', 'Fontsize', 10);
subplot(3, 1, 1);
plot(total_t, Endp_x);xlabel('t (s)'),ylabel('x (mm)');
for n = 1: length(eva_t)
    xline(cumeva_t(n) + letter_t*(n - 1),'--b');xline(cumeva_t(n) + letter_t*n,'--r');
end
subplot(3, 1, 2);
plot(total_t, Endp_y);xlabel('t (s)'),ylabel('y (mm)');
for n = 1: length(eva_t)
    xline(cumeva_t(n) + letter_t*(n - 1),'--b');xline(cumeva_t(n) + letter_t*n,'--r');
end
subplot(3, 1, 3);
plot(total_t, Endp_z);xlabel('t (s)'),ylabel('z (mm)');
for n = 1: length(eva_t)
    xline(cumeva_t(n) + letter_t*(n - 1),'--b');xline(cumeva_t(n) + letter_t*n,'--r');
end
%% velocity
f2 = figure;
f2.Position(3:4) = [850 420];
sgtitle('The velocity of the end-effector', 'Fontsize', 10);
subplot(3, 1, 1);
plot(total_t, Endv_x);xlabel('t (s)'),ylabel('v_x (mm/s)');
for n = 1: length(eva_t)
    xline(cumeva_t(n) + letter_t*(n - 1),'--b');xline(cumeva_t(n) + letter_t*n,'--r');
end
subplot(3, 1, 2);
plot(total_t, Endv_y);xlabel('t (s)'),ylabel('v_y (mm/s)');
for n = 1: length(eva_t)
    xline(cumeva_t(n) + letter_t*(n - 1),'--b');xline(cumeva_t(n) + letter_t*n,'--r');
end
subplot(3, 1, 3);
plot(total_t, Endv_z);xlabel('t (s)'),ylabel('v_z (mm/s)');
for n = 1: length(eva_t)
    xline(cumeva_t(n) + letter_t*(n - 1),'--b');xline(cumeva_t(n) + letter_t*n,'--r');
end
%% acceleration
f3 = figure;
f3.Position(3:4) = [850 420];
sgtitle('The acceleration of the end-effector', 'Fontsize', 10);
subplot(3, 1, 1);
plot(total_t, Enda_x);xlabel('t (s)'),ylabel('a_x (mm/s^2)');
for n = 1: length(eva_t)
    xline(cumeva_t(n) + letter_t*(n - 1),'--b');xline(cumeva_t(n) + letter_t*n,'--r');
end
subplot(3, 1, 2);
plot(total_t, Enda_y);xlabel('t (s)'),ylabel('a_y (mm/s^2)');
for n = 1: length(eva_t)
    xline(cumeva_t(n) + letter_t*(n - 1),'--b');xline(cumeva_t(n) + letter_t*n,'--r');
end
subplot(3, 1, 3);
plot(total_t, Enda_z);xlabel('t (s)'),ylabel('a_z (mm/s^2)');
for n = 1: length(eva_t)
    xline(cumeva_t(n) + letter_t*(n - 1),'--b');xline(cumeva_t(n) + letter_t*n,'--r');
end
end

