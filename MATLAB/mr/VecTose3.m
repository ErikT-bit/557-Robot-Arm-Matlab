function se3mat = VecTose3(V)
% *** CHAPTER 3: RIGID-BODY MOTIONS ***
% Converts a spatial velocity vector into a 4x4 matrix in se(3)
% V = [omg; v]
se3mat = [ VecToso3(V(1:3)), V(4:6);
           0 0 0            , 0     ];
end
