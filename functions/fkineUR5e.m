function [T, JointPos] = fkineUR5e(theta, d, a, alpha) 
% theta, d, a, alpha: dh parameters
% T0
T{1} = eye(4);
JointPos(1, :) = T{1}(1:3, 4)';
% compute T01, T02 ... T0n
for n = 2: length(theta) + 1
    k = n - 1;
    % I*T01, I*T01*T12 ...
    T{n} = T{n-1}*T_param(theta(k), d(k), a(k), alpha(k));
    JointPos(n, :) = T{n}(1:3, 4)';
end

end

function T = T_param(theta, d, a, alpha)
T = [cos(theta), -sin(theta)*cos(alpha), sin(theta)*sin(alpha) , a*cos(theta);
     sin(theta), cos(theta)*cos(alpha) , -cos(theta)*sin(alpha), a*sin(theta);
     0         , sin(alpha)            , cos(alpha)            , d           ;
     0         , 0                     , 0                     , 1            ];
end
