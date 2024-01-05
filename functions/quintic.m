function [t, q, v, a] = quintic(t_array, q_array, v_array, a_array, eta)
% eta: sampling points
% initial state
q = q_array(1);
t = t_array(1);
v = v_array(1);
a = a_array(1);
t = []; q = []; v = []; a = [];
for i = 1: 1 : length(q_array) - 1
    T = t_array(i+1) - t_array(i);
    % factors
    a0 = q_array(i);
    a1 = v_array(i);
    a2 = a_array(i)/2;
    a3 = (20*q_array(i+1)-20*q_array(i)-(8*v_array(i+1)+12*v_array(i))*T-(3*a_array(i)-a_array(i+1))*T^2)/(2*T^3);
    a4 = (30*q_array(i)-30*q_array(i+1)+(14*v_array(i+1)+16*v_array(i))*T+(3*a_array(i)-2*a_array(i+1))*T^2)/(2*T^4);
    a5 = (12*q_array(i+1)-12*q_array(i)-(6*v_array(i+1)+6*v_array(i))*T-(a_array(i)-a_array(i+1))*T^2)/(2*T^5);
    % compute q, v and a
    ti = t_array(i): eta: t_array(i+1);
    qi = a0+a1*(ti-t_array(i))+a2*(ti-t_array(i)).^2+a3*(ti-t_array(i)).^3+a4*(ti-t_array(i)).^4+a5*(ti-t_array(i)).^5;
    vi = a1+2*a2*(ti-t_array(i))+3*a3*(ti-t_array(i)).^2+4*a4*(ti-t_array(i)).^3+5*a5*(ti-t_array(i)).^4;
    ai = 2*a2+6*a3*(ti-t_array(i))+12*a4*(ti-t_array(i)).^2+20*a5*(ti-t_array(i)).^3;
    t = [t, ti(2:end)];
    q = [q, qi(2:end)];
    v = [v, vi(2:end)];
    a = [a, ai(2:end)];
end
