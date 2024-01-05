function [points] = averageDis(point1, point2, L)
% L: set the interpolation interval
N = fix(norm(point2 - point1)/L);
dp = (point2 - point1)./N;
points = [];
for n = 1: N + 1
    points = [points; point1 + dp.*(n - 1)];
end
end
