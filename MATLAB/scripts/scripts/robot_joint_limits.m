function limits = robot_joint_limits()
% ROBOT_JOINT_LIMITS  Zero-centered joint limits from the ROS/MoveIt URDF.
%
% These are the ACTUAL operational limits of the physical arm, extracted
% from the me557_pen.urdf used by MoveIt in the Otis-Spunkmeyer/ME_557_Robot
% repository.  The joint order matches the 5 actuated joints used in
% main_writeboard_demo.m:
%
%   [RotatingBaseJoint, ShoulderJoint, ElbowJoint, WristJoint, PenJoint]
%
% which correspond to the MoveIt joint names:
%
%   [Motor1_joint, Motor2_L, Motor4_elb, Motor5_wr, Joint_EE]
%
% USAGE:
%   limits = robot_joint_limits();
%   theta  = IKinSpace(...);
%   theta  = clamp_to_limits(theta, limits);

%                     Base      Shoulder  Elbow     Wrist     EE
limits.lower     = [ -0.87265; -1.04720; -1.87623; -0.95995; -1.17810 ];
limits.upper     = [  0.87265;  1.04720;  1.87623;  0.95995;  1.17810 ];

% Readable names (for display / debugging)
limits.names = ["RotatingBase"; "Shoulder"; "Elbow"; "Wrist"; "PenEE"];

% Degrees (convenience)
limits.lower_deg = limits.lower * (180/pi);
limits.upper_deg = limits.upper * (180/pi);
end
