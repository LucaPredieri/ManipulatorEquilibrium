function [S] = ComputeRigidBodyJ(v)
% Computing the matrix S for the wrenches away from the center of mass.

S = zeros(6,6);
skew = zeros(3,3);
skew = [0 -v(3) v(2);v(3) 0 -v(1);-v(2) v(1) 0];
S = [eye(3), zeros(3,3); skew , eye(3)];


end
