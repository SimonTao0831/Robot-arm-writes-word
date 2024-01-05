function [tf] = allocationTime(points, total_t)
dis = zeros(1, size(points, 1) - 1);
% set the cost time of one character, return each segment's time 
for n = 1: size(points, 1) - 1
    % the distance between two interpolation points
    dis(n) = norm(points(n, :) - points(n + 1, :));
end
total_dis = sum(dis); % total distance
tf = roundn((dis./total_dis)*total_t, 0); % each segments' time

end
