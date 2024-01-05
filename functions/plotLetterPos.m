function [] = plotLetterPos(Endeffector, tp, str)
Endp_x = Endeffector(:, 1)*1000;
Endp_y = Endeffector(:, 2)*1000;
Endp_z = Endeffector(:, 3)*1000;
% Endv_x = [0; diff(Endp_x)];
% Endv_y = [0; diff(Endp_y)];
% Endv_z = [0; diff(Endp_z)];
% Enda_x = [0; diff(Endv_x)];
% Enda_y = [0; diff(Endv_y)];
% Enda_z = [0; diff(Endv_z)];
f = figure();
f.Position(3:4) = [450 350];
sgtitle(str, 'Fontsize', 10);
subplot(3, 1, 1);
plot(tp, Endp_x);xlabel('t (s)'),ylabel('x (mm)');
subplot(3, 1, 2);
plot(tp, Endp_y);xlabel('t (s)'),ylabel('y (mm)');
subplot(3, 1, 3);
plot(tp, Endp_z);xlabel('t (s)'),ylabel('z (mm)');
end
