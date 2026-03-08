function limits = robot_joint_limits()
% robot_joint_limits
% ROS/MoveIt zero-centered operational limits (radians).
% Order: [base; shoulder; elbow; wrist; ee]

limits = [ ...
   -0.87265, +0.87265;   % base   (-50  to +50 deg)
   -1.04720, +1.04720;   % shoulder (-60 to +60 deg)
   -1.87623, +1.87623;   % elbow (-107.5 to +107.5 deg)
   -0.95995, +0.95995;   % wrist (-55 to +55 deg)
   -1.17810, +1.17810];  % ee    (-67.5 to +67.5 deg)
end