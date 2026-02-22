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
nJ = numel(jointNames);
fprintf('    OK — %d joints loaded.\n\n', nJ);

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
for j = 1:nJ
    S = robot.Slist(:,j);
    fprintf('    J%-5d %+8.4f %+8.4f %+8.4f | %+8.4f %+8.4f %+8.4f\n', ...
        j, S);
end
fprintf('\n');

for j = 1:nJ
    wNorm = norm(robot.Slist(1:3,j));
    if abs(wNorm - 1.0) > 0.01
        fprintf('    WARNING: ||w_%d|| = %.4f (expected 1.0 for revolute)\n', j, wNorm);
    end
end

% =====================================================================
%  4.  Joint limits
% =====================================================================
fprintf('[4] Joint limits:\n');
jlim = get_joint_limits(nJ, jointNames);
for j = 1:nJ
    fprintf('    J%d %-14s  %+7.2f deg to %+7.2f deg  (%+.4f to %+.4f rad)\n', ...
        j, jlim.names{j}, rad2deg(jlim.lower(j)), rad2deg(jlim.upper(j)), ...
        jlim.lower(j), jlim.upper(j));
end
fprintf('\n');

% =====================================================================
%  5.  FK verification at home
% =====================================================================
fprintf('[5] FK verification at home (theta = [0,0,0,0,0]):\n');
T0 = FKinSpace(robot.M, robot.Slist, robot.home);
fprintf('    FKinSpace result:\n');
print_T(T0);
fk_matches = max(abs(T0(:) - robot.M(:))) < 1e-10;
fprintf('    Matches M?  ');
if fk_matches
    fprintf('YES (max error = %.1e)\n\n', max(abs(T0(:)-robot.M(:))));
else
    fprintf('NO! max error = %.6e\n\n', max(abs(T0(:)-robot.M(:))));
end

% =====================================================================
%  6.  FK at several test poses
% =====================================================================
fprintf('[6] FK at test poses:\n');
testThetas = {
    zeros(nJ,1),                        'Home (all zeros)';
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
thetaTest = [0.3; 0.4; -0.8; 0.2; 0.1];
T_target = FKinSpace(robot.M, robot.Slist, thetaTest);
fprintf('    Forward: theta = [%s] deg\n', num2str(rad2deg(thetaTest'),'%+.1f  '));
fprintf('    FK tip  = [%.5f, %.5f, %.5f]\n', T_target(1:3,4));

rng(42);
theta0 = thetaTest + 0.1*randn(nJ,1);
[thetaIK, ikSuccess] = IKinSpace(robot.Slist, robot.M, T_target, theta0, 1e-4, 1e-4);

if ikSuccess
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
    manipVal = sqrt(max(0, det(J*J')));
    rankJ = rank(J, 1e-6);
    fprintf('    %-30s  rank=%d  manip=%.6f', nm, rankJ, manipVal);
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
tips = zeros(3, N);
for i = 1:N
    thRand = jlim.lower + (jlim.upper - jlim.lower) .* rand(nJ,1);
    T = FKinSpace(robot.M, robot.Slist, thRand);
    tips(:,i) = T(1:3,4);
end
fprintf('    X range: [%.4f, %.4f] m\n', min(tips(1,:)), max(tips(1,:)));
fprintf('    Y range: [%.4f, %.4f] m\n', min(tips(2,:)), max(tips(2,:)));
fprintf('    Z range: [%.4f, %.4f] m\n', min(tips(3,:)), max(tips(3,:)));
maxR = max(sqrt(sum(tips.^2, 1)));
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
if fk_matches, fprintf('  FK at home matches: YES\n');
else,          fprintf('  FK at home matches: NO\n'); end
if ikSuccess,  fprintf('  IK round-trip:      PASS\n');
else,          fprintf('  IK round-trip:      FAIL\n'); end
fprintf('  # joints:           %d\n', nJ);
fprintf('  Tip at home:        [%.4f, %.4f, %.4f] m\n', tipHome);
fprintf('  Max reach:          %.4f m\n', maxR);
fprintf('============================================================\n');
fprintf('\n  Run  visualize_robot  for the interactive 3-D viewer.\n\n');
end

% =========================================================================
function jlim = get_joint_limits(nJ, jointNames)
% Try robot_joint_limits(), fall back to hard-coded ROS values.
jlim = struct('lower', [], 'upper', [], 'names', {{}});

loaded = false;
try
    tmp = robot_joint_limits();       %#ok — may not exist
    if isstruct(tmp) && isfield(tmp,'lower') && isfield(tmp,'upper')
        jlim.lower = tmp.lower(:);
        jlim.upper = tmp.upper(:);
        if isfield(tmp,'names')
            if iscell(tmp.names)
                jlim.names = tmp.names;
            else
                jlim.names = cellstr(tmp.names);
            end
        end
        loaded = true;
        fprintf('    Source: robot_joint_limits.m\n');
    end
catch
    % robot_joint_limits not on path — fine, use defaults below
end

if ~loaded
    fprintf('    Source: hard-coded ROS/MoveIt values (robot_joint_limits.m not found)\n');
    jlim.lower = [ -0.87265; -1.04720; -1.87623; -0.95995; -1.17810 ];
    jlim.upper = [  0.87265;  1.04720;  1.87623;  0.95995;  1.17810 ];
end

if isempty(jlim.names) || numel(jlim.names) ~= nJ
    jlim.names = {'RotatingBase','Shoulder','Elbow','Wrist','PenEE'};
end
end

% =========================================================================
function print_T(T)
for r = 1:4
    fprintf('    [%+9.5f %+9.5f %+9.5f %+9.5f]\n', T(r,:));
end
end