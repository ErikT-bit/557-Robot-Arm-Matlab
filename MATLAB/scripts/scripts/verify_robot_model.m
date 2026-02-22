function verify_robot_model()
% VERIFY_ROBOT_MODEL  Non-interactive diagnostics — NO Robotics Toolbox.
%
% Loads the URDF, checks FK/IK round-trip, prints screw axes, joint
% limits, Jacobian, and tests several known poses.  Run this first to
% confirm everything is wired correctly before using the interactive viewer.
%
% USAGE (from MATLAB/scripts/):
%   verify_robot_model

clc;

% --- paths ---
thisFile   = mfilename('fullpath');
scriptsDir = fileparts(thisFile);
matlabDir  = fileparts(scriptsDir);
addpath(genpath(fullfile(matlabDir, 'mr')));
addpath(scriptsDir);

urdfPath   = fullfile(matlabDir, 'simple_robot_v2.urdf');
baseLink   = "FixedBase";
tipLink    = "PenTipLink";
jointNames = ["RotatingBaseJoint","ShoulderJoint","ElbowJoint", ...
              "WristJoint","PenJoint"];

fprintf('============================================================\n');
fprintf('  ME557 Robot Model Verification (No Robotics Toolbox)\n');
fprintf('============================================================\n\n');

% =====================================================================
%  1.  Load model
% =====================================================================
fprintf('[1] Loading URDF: %s\n', urdfPath);
if ~isfile(urdfPath)
    error('URDF not found at %s', urdfPath);
end
robot = robot_model_from_urdf(urdfPath, baseLink, tipLink, jointNames);
fprintf('    OK — %d joints loaded.\n\n', numel(jointNames));

% =====================================================================
%  2.  Home transform M
% =====================================================================
fprintf('[2] Home transform M (base -> %s at theta=0):\n', tipLink);
print_T(robot.M);

tipHome = robot.M(1:3,4);
fprintf('    Tip at home: [%.5f, %.5f, %.5f] m\n', tipHome);
fprintf('    Height above base: %.4f m  (%.2f inches)\n\n', ...
    tipHome(3), tipHome(3)/0.0254);

% =====================================================================
%  3.  Screw axes
% =====================================================================
fprintf('[3] Screw axes (space frame, at home):\n');
fprintf('    %6s  %8s %8s %8s | %8s %8s %8s\n', ...
    'Joint', 'w1','w2','w3', 'v1','v2','v3');
fprintf('    %s\n', repmat('-',1,60));
for j = 1:size(robot.Slist,2)
    S = robot.Slist(:,j);
    fprintf('    J%-5d %+8.4f %+8.4f %+8.4f | %+8.4f %+8.4f %+8.4f\n', ...
        j, S);
end
fprintf('\n');

% Sanity: screw axes should have unit-norm angular part for revolute
for j = 1:size(robot.Slist,2)
    wNorm = norm(robot.Slist(1:3,j));
    if abs(wNorm - 1.0) > 0.01
        fprintf('    WARNING: ||w_%d|| = %.4f (expected 1.0 for revolute)\n', j, wNorm);
    end
end

% =====================================================================
%  4.  Joint limits
% =====================================================================
fprintf('[4] Joint limits:\n');
try
    limits = robot_joint_limits();
    fprintf('    Loaded from robot_joint_limits.m (ROS zero-centered)\n');
    for j = 1:numel(limits.lower)
        fprintf('    J%d %-14s  %+7.2f° to %+7.2f°  (%+.4f to %+.4f rad)\n', ...
            j, limits.names(j), rad2deg(limits.lower(j)), rad2deg(limits.upper(j)), ...
            limits.lower(j), limits.upper(j));
    end
catch
    fprintf('    robot_joint_limits.m NOT FOUND — add it to scripts/\n');
    limits.lower = -pi*ones(5,1);
    limits.upper =  pi*ones(5,1);
end
fprintf('\n');

% =====================================================================
%  5.  FK verification at home
% =====================================================================
fprintf('[5] FK verification at home (theta = [0,0,0,0,0]):\n');
T0 = FKinSpace(robot.M, robot.Slist, robot.home);
fprintf('    FKinSpace result:\n');
print_T(T0);
fprintf('    Matches M?  ');
if max(abs(T0(:) - robot.M(:))) < 1e-10
    fprintf('YES (max error = %.1e)\n\n', max(abs(T0(:)-robot.M(:))));
else
    fprintf('NO! max error = %.6e\n\n', max(abs(T0(:)-robot.M(:))));
end

% =====================================================================
%  6.  FK at several test poses
% =====================================================================
fprintf('[6] FK at test poses:\n');
testThetas = {
    zeros(5,1),                         'Home (all zeros)';
    [0.5; 0; 0; 0; 0],                 'Base +28.6 deg only';
    [0; 0.5; 0; 0; 0],                 'Shoulder +28.6 deg only';
    [0; 0; -1.0; 0; 0],                'Elbow -57.3 deg only';
    [0; 0.3; -0.5; 0.2; 0],            'Mixed pose';
};

for t = 1:size(testThetas,1)
    th = testThetas{t,1};
    nm = testThetas{t,2};
    T = FKinSpace(robot.M, robot.Slist, th);
    pos = T(1:3,4);
    fprintf('    %-30s -> tip = [%+.4f, %+.4f, %+.4f] m\n', nm, pos);
end
fprintf('\n');

% =====================================================================
%  7.  IK round-trip test
% =====================================================================
fprintf('[7] IK round-trip test:\n');
% Pick a known pose via FK, then solve IK back, compare
thetaTest = [0.3; 0.4; -0.8; 0.2; 0.1];
T_target = FKinSpace(robot.M, robot.Slist, thetaTest);
fprintf('    Forward: theta = [%s] deg\n', num2str(rad2deg(thetaTest'),'%+.1f  '));
fprintf('    FK tip  = [%.5f, %.5f, %.5f]\n', T_target(1:3,4));

% Use a slightly perturbed initial guess
theta0 = thetaTest + 0.1*randn(5,1);
[thetaIK, success] = IKinSpace(robot.Slist, robot.M, T_target, theta0, 1e-4, 1e-4);

if success
    T_check = FKinSpace(robot.M, robot.Slist, thetaIK);
    posErr = norm(T_check(1:3,4) - T_target(1:3,4));
    fprintf('    IK solved: theta = [%s] deg\n', num2str(rad2deg(thetaIK'),'%+.1f  '));
    fprintf('    IK tip  = [%.5f, %.5f, %.5f]\n', T_check(1:3,4));
    fprintf('    Position error: %.2e m', posErr);
    if posErr < 1e-3
        fprintf('  PASS\n');
    else
        fprintf('  LARGE ERROR!\n');
    end
else
    fprintf('    IK FAILED to converge. Check maxiterations in IKinSpace.m\n');
    fprintf('    (Default is 20 — try increasing to 100 for harder problems.)\n');
end
fprintf('\n');

% =====================================================================
%  8.  Jacobian analysis
% =====================================================================
fprintf('[8] Jacobian analysis:\n');
for t = 1:size(testThetas,1)
    th = testThetas{t,1};
    nm = testThetas{t,2};
    J = JacobianSpace(robot.Slist, th);
    manip = sqrt(max(0, det(J*J')));
    rankJ = rank(J, 1e-6);
    fprintf('    %-30s  rank=%d  manip=%.6f', nm, rankJ, manip);
    if rankJ < min(size(J))
        fprintf('  ** SINGULAR **');
    end
    fprintf('\n');
end
fprintf('\n');

% =====================================================================
%  9.  Workspace sampling
% =====================================================================
fprintf('[9] Workspace estimate (random sampling, 5000 points):\n');
N = 5000;
rng(42);
lo = limits.lower;
hi = limits.upper;
tips = zeros(3, N);
for i = 1:N
    thRand = lo + (hi - lo) .* rand(5,1);
    T = FKinSpace(robot.M, robot.Slist, thRand);
    tips(:,i) = T(1:3,4);
end
fprintf('    X range: [%.4f, %.4f] m\n', min(tips(1,:)), max(tips(1,:)));
fprintf('    Y range: [%.4f, %.4f] m\n', min(tips(2,:)), max(tips(2,:)));
fprintf('    Z range: [%.4f, %.4f] m\n', min(tips(3,:)), max(tips(3,:)));
maxR = max(vecnorm(tips));
fprintf('    Max reach from base: %.4f m (%.2f in)\n', maxR, maxR/0.0254);
fprintf('\n');

% Plot workspace cloud
figure('Name','Workspace Cloud','NumberTitle','off');
scatter3(tips(1,:), tips(2,:), tips(3,:), 2, tips(3,:), 'filled');
hold on; axis equal; grid on; colorbar;
xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
title('ME557 Reachable Workspace (5000 random samples)');
plot3(0,0,0,'k^','MarkerSize',14,'MarkerFaceColor','k');
view(135,25);

% =====================================================================
%  10.  Summary
% =====================================================================
fprintf('============================================================\n');
fprintf('  SUMMARY\n');
fprintf('============================================================\n');
fprintf('  URDF loaded:        YES\n');
fprintf('  FK at home matches: %s\n', iif(max(abs(T0(:)-robot.M(:)))<1e-10,'YES','NO'));
fprintf('  IK round-trip:      %s\n', iif(success,'PASS','FAIL'));
fprintf('  # joints:           %d\n', size(robot.Slist,2));
fprintf('  Tip at home:        [%.4f, %.4f, %.4f] m\n', tipHome);
fprintf('  Max reach:          %.4f m\n', maxR);
fprintf('  Joint limits:       %s\n', iif(exist('robot_joint_limits','file')>0,'LOADED','MISSING'));
fprintf('============================================================\n');
fprintf('\n  Run  visualize_robot  for the interactive 3-D viewer.\n\n');
end

% --- Utility ---
function print_T(T)
    for r = 1:4
        fprintf('    [%+9.5f %+9.5f %+9.5f %+9.5f]\n', T(r,:));
    end
end

function s = iif(cond, yes, no)
    if cond, s = yes; else, s = no; end
end