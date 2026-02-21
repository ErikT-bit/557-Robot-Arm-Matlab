function T = MatrixExp6(se3mat)
% *** CHAPTER 3: RIGID-BODY MOTIONS ***
omgtheta = so3ToVec(se3mat(1:3, 1:3));
if NearZero(norm(omgtheta))
    % Pure translation
    T = [eye(3), se3mat(1:3, 4);
         0 0 0, 1];
else
    theta = norm(omgtheta);
    omgmat = se3mat(1:3, 1:3) / theta;
    R = MatrixExp3(se3mat(1:3, 1:3));
    G = eye(3) * theta + (1 - cos(theta)) * omgmat + (theta - sin(theta)) * (omgmat * omgmat);
    p = G * (se3mat(1:3, 4) / theta);
    T = [R, p;
         0 0 0, 1];
end
end
