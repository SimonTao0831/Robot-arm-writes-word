function [theta] = ikineUR5e(T, d, a)
% T: transformation matrix of goal 
% d, a: dh parameters
% known transformation matrix
nx = T(1, 1); ox = T(1, 2); ax = T(1, 3); px = T(1, 4);
ny = T(2, 1); oy = T(2, 2); ay = T(2, 3); py = T(2, 4);
nz = T(3, 1); oz = T(3, 2); az = T(3, 3); pz = T(3, 4);
% a
a2 = a(2);
a3 = a(3);
% d
d1 = d(1);
d4 = d(4);
d5 = d(5);
d6 = d(6);
% positive and negative signs, 8 cases
f = [1 1 1;
    1 1 -1;
    1 -1 1;
    1 -1 -1;
    -1 1 1;
    -1 1 -1;
    -1 -1 1;
    -1 -1 -1];
% theta
theta = [];

% loop to solve theta
for k = 1: 8
    try % if no solution, then skip
    % theta 1, f(k, 1)
    m1 = d6*ay - py;
    n1 = ax*d6 - px;
    t1 = atan2(m1, n1) - atan2(d4, f(k, 1)*sqrt(m1^2 + n1^2 - d4^2));
    
    % theta 5, f(k, 2)
    t5 = f(k, 2)*acos(ax*sin(t1) - ay*cos(t1));
    
    % theta 6
    m6 = nx*sin(t1) - ny*cos(t1);
    n6 = ox*sin(t1) - oy*cos(t1);
    t6 = atan2(m6, n6) - atan2(sin(t5), 0);
    
    % theta 3, f(k, 3)
    m3 = d5*(sin(t6)*(nx*cos(t1) + ny*sin(t1)) + cos(t6)*(ox*cos(t1) + oy*sin(t1))) - d6*(ax*cos(t1) + ay*sin(t1)) + px*cos(t1) + py*sin(t1);
    n3 = pz - d1 - az*d6 + d5*(oz*cos(t6) + nz*sin(t6));
    t3 = f(k, 3)*acos((m3^2 + n3^2 - a2^2 - a3^2)/(2*a2*a3));
    
    % theta 2
    s2 = ((a3*cos(t3) + a2)*n3 - a3*sin(t3)*m3)/(a2^2 + a3^2 + 2*a2*a3*cos(t3));
    c2 = (m3 + a3*sin(t3)*s2)/(a3*cos(t3) + a2);
    t2 = atan2(s2, c2);
    
    % theta 4
    s234 = -sin(t6)*(nx*cos(t1) + ny*sin(t1)) - cos(t6)*(ox*cos(t1) + oy*sin(t1));
    c234 = oz*cos(t6) + nz*sin(t6);
    t4 = atan2(s234, c234) - t2 - t3;
    
    % solution
    sol = [t1 t2 t3 t4 t5 t6];
    theta = [theta; sol];
    end
end

% ensure the range in [-pi, pi]
for i = 1: numel(theta)
    t = theta(i);
    if t > pi
        theta(i) = t - 2*pi;
    elseif t < -pi
        theta(i) = t + 2*pi;
    end
end

end
