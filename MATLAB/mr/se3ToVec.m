function V = se3ToVec(se3mat)
% se3ToVec: Converts a 4x4 se(3) matrix into a 6x1 twist vector.
%
% Input:
%   se3mat - 4x4 matrix of the form [ [so3]  v;
%                                     0     0 ]
% Output:
%   V - 6x1 vector [omega; v]

V = [se3mat(3,2); se3mat(1,3); se3mat(2,1); se3mat(1:3,4)];
end
