function [tp, vp] = lettertrajectory(F, T, dt)
% % each letter's time
% T = 20; % s
% % waypoint
% F = [-175 100 0
%      -275 100 0
%      -275 -100 0
%      -275 20 0
%      -175 20 0];
% homogeneous transformation matrix
R = eulerR(pi/2, 0, pi/2);
p = [-500, -150, 500]';
H = [R p; 0 0 0 1];
Ft = H*[F'; ones(1, size(F, 1))];
Fp = Ft(1:3, :);

% each length, mm
for n = 2: size(F, 1)
    l(n-1) = norm(Fp(:, n) - Fp(:, n-1)); % distance
end
% total length
L = sum(l);
% desired average velocity
v = L/T; % mm/s
% time spent per segment
t = l/v;
%% velocity vector
vv = [];
for n = 2: size(F, 1)
    vv = [vv (Fp(:, n) - Fp(:, n-1))./roundn(t(n-1), -1)];
end
dx = [vv; zeros(3, size(vv, 2))];
%% time stamp
tp = [];
vp = [];
point = 0;
for n = 1: length(t)
    if ~isempty(tp)
      point = tp(end);
    end
    stp = 0: dt: t(n); % this period time
    % velocity stamp
    for m = 1: length(stp) - 1
        vpp = dx(:, n).*(stp(m+1) - stp(m));
        vp = [vp vpp];
    end
    % timestamp
    tp = [tp point + stp(2: end)];
end
end

