function [J] = mJacobian(T)
for n = 1: numel(T) - 1
    br = cross(T{n}(1:3, 1:3)*[0; 0; 1], T{end}(1:3, 4) - T{n}(1:3, 4));
    b = T{n}(1:3, 1:3)*[0; 0; 1];
    J(:, n) = [br; b];
end
end
