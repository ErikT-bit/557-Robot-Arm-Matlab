function T = FKinBody(M, Blist, thetalist)
% *** CHAPTER 4: FORWARD KINEMATICS (Body Frame) ***
% Takes:
%   M:        4x4 home configuration of the end-effector (SE(3))
%   Blist:    6xn matrix of joint screw axes in the end-effector frame
%             at the home position (each column is a Bi)
%   thetalist: n-vector of joint coordinates (angles/displacements)
% Returns:
%   T: 4x4 end-effector configuration in SE(3) using the body PoE formula
%
% Example:
% clear; clc;
% M = [[-1, 0, 0, 0]; [0, 1, 0, 6]; [0, 0, -1, 2]; [0, 0, 0, 1]];
% Blist = [[0; 0; -1; 2; 0; 0], [0; 0; 0; 0; 1; 0], [0; 0; 1; 0; 0; 0.1]];
% thetalist = [pi/2; 3; pi];
% T = FKinBody(M, Blist, thetalist)

    % ---- basic input shaping (robust to row/column vectors) ----
    thetalist = real(double(thetalist(:)));   % force n×1 real double
    n = length(thetalist);

    % ---- optional sanity checks (comment out if you want ultra-minimal) ----
    if ~isequal(size(M), [4 4])
        error('FKinBody:BadM', 'M must be 4x4.');
    end
    if size(Blist,1) ~= 6
        error('FKinBody:BadBlist', 'Blist must be 6xn.');
    end
    if size(Blist,2) ~= n
        error('FKinBody:SizeMismatch', ...
              'Blist must have %d columns to match thetalist length.', n);
    end

    % ---- body-frame product of exponentials ----
    T = M;
    for i = 1:n
        T = T * MatrixExp6(VecTose3(Blist(:, i) * thetalist(i)));
    end
end
