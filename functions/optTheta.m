function [theta_opt] = optTheta(theta_pre, theta_sol)
% theta_pre: previous theta
% theta_now: theta of solutions
% constraint1: theta2 only clockwise
theta1 = theta_sol(theta_sol(:, 2) < 0, :);
% theta1 = theta_sol;

% constraint2: theta4 only clockwise
% theta2 = theta1(theta1(:, 4) < 0, :); % somtimes will cause bug
theta2 = theta1;

% constraint3: minimum angle change
sol_size = size(theta2, 1); % existence of solutions
if sol_size > 1 % more tha 2 solutions
    theta_diff = theta2 - repmat(theta_pre, sol_size, 1);
    [~, idx] = min(sum(theta_diff.^2, 2));
    theta_opt = theta2(idx, :);
else
    theta_opt = theta2;
end
end
