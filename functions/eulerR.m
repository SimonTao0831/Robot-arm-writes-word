function [Rzyx] = eulerR(q1, q2, q3)
Rx = [1, 0, 0;
      0, cos(q1), -sin(q1);
      0, sin(q1), cos(q1)];
Ry = [cos(q2), 0, sin(q2);
      0, 1, 0;
      -sin(q2), 0, cos(q2)];
Rz = [cos(q3), -sin(q3), 0;
      sin(q3), cos(q3), 0;
      0, 0, 1];
Rzyx = Rz*Ry*Rx;
end

