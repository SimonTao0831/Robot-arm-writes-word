function [Fp] = transfWord(F, R, p)
% homogeneous transformation matrix
H = [R p; 0 0 0 1];
Ft = H*[F'; ones(1, size(F, 1))];
Fp = Ft(1:3, :)';
end

