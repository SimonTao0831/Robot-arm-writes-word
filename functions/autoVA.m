function [t, v_f, a_f] = autoVA(onejoint, t0, tf, acct, dacct, accp, daccp)
% onejoint theta
% t0: start time
% tf: end time
% acct: accelerate time
% dacct: decelerate time
% accp: interpolation segments are used to accelerate
% daccp: interpolation segments are used to decelerate
%% dividing acceleration and deceleration segments
n = length(onejoint); % total interpolation points
nacc = round(accp*n); % used to accelerate
ndacc = round(daccp*n); % used to decelerate
% nacc = 1; % used to accelerate
% ndacc = 1; % used to decelerate
nave = n - nacc - ndacc;
%% set corresponding time
total_t = tf - t0;
t0c = t0 + total_t*acct;
tfc = tf - total_t*dacct;
tacc = t0: (t0c - t0)/(nacc - 1): t0c;
tave = t0c: (tfc - t0c)/(nave + 1): tfc;
tave(1) = []; tave(end) = [];
tdacc = tfc: (tf - tfc)/(ndacc - 1): tf;
t = [tacc, tave, tdacc];
%% velocity
dq = diff(onejoint);
dt = diff(t');
slope_q = dq./dt;
%
v = zeros(n - 2, 1);
for n1 = 1: n - 2
    v(n1) = (slope_q(n1) + slope_q(n1 + 1))/2;
end
v_f = [0; v; 0];
%% acceleration
dv = diff(v_f);
slope_v = dv./dt;
%
a = zeros(n - 2, 1);
for n2 = 1: n - 2
    a(n2) = (slope_v(n2) + slope_v(n2 + 1))/2;
end
a_f = [0; a; 0];
% transpose to 1xN
t = t';
v_f = v_f';
a_f = a_f';
end

