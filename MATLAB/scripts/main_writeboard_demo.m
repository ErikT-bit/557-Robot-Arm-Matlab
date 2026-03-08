function main_writeboard_demo()
% main_writeboard_demo
% Flow:
%   1) Go HOME (skip if already within +/-5 deg)
%   2) Plane capture
%   3) Move to permanent waypoint1
%   4) Sketch capture
%   5) Scale / center
%   6) Strokes -> task space
%   7) Solve IK
%   8) Execute drawing
%   9) Return HOME
%
% Current behavior:
%   - waypoint1 is permanent
%   - NO extra press offset is added during plane fit
%   - writing plane is offset by boardBackoff_m
%   - pen-up pulls 2 inches back from the board
%   - if already close to HOME, skip the long HOME move

clc; close all;

% ---- PATHS ----
thisFile = mfilename('fullpath');
scriptsDir = fileparts(thisFile);
matlabDir  = fileparts(scriptsDir);

addpath(genpath(fullfile(matlabDir,"mr")));
addpath(genpath(scriptsDir));

% ---- URDF / MODEL ----
urdfPath = fullfile(matlabDir, "simple_robot_v2.urdf");
baseLink = "FixedBase";
tipLink  = "PenTipLink";
jointNames5 = ["RotatingBaseJoint","ShoulderJoint","ElbowJoint","WristJoint","PenJoint"];

robot = robot_model_from_urdf(urdfPath, baseLink, tipLink, jointNames5);
fprintf("Loaded URDF: %s\n", urdfPath);

% ---- CALIB + HW ----
hw = robot_hw_rb150_calibrated_6motor("COM13", 1000000);
cal = servo_calibration(); %#ok<NASGU>

% ---- VIEWER ----
theta_now0 = hw.readJoints();
viewer = visualize_robot('robot',robot,'urdf',urdfPath,'theta0',theta_now0, ...
    'title','ME557 Writeboard Live Viewer');

% ---- SETTINGS ----
theta_home = deg2rad([0 0 0 0 0]);

% ---- PERMANENT WAYPOINT1 ----
use_waypoint1 = true;
theta_waypoint1 = [ ...
     0.0819; ...
    -0.5462; ...
    -1.2130; ...
    -0.0051; ...
    -0.3628];

% ---- BOARD / DRAWING SETTINGS ----
planePressOffset_m = 0.0;       % no extra plane press offset
boardBackoff_m     = -0.0035;     % 5 mm board offset
penUpHeight_m      = 2 * 0.0254; % 2 inches pull-back from board

desiredTextHeight_in = 3.0;
desiredTextHeight_m  = desiredTextHeight_in * 0.0254;

boardSize_in = 16;
boardSize_m  = boardSize_in * 0.0254;

ev = 1e-3;
maxIters = 200;

% ---- MOTION TUNING ----
dt_home   = 0.05;
t_home_s  = 10.0 / 1.7;
maxDegPerSec_home = 7 * 1.7;

dt_approach   = 0.045;
t_approach_s  = 3.0;
maxDegPerSec_approach = 9;

dt_exec   = 0.035;
maxDegPerSec_draw = 14;

homeTol_deg = 5;

% =====================================================================
% STEP 1: GO HOME
% =====================================================================
fprintf("\n=== STEP 1: GO HOME ===\n");
disp("Torque ON...");
hw.torqueOn();
pause(0.4);

disp("Reading current joints...");
theta_start = hw.readJoints();
viewer.update(theta_start);

fprintf("  Current (deg): [%s]\n", joint_deg_str(theta_start));
fprintf("  Home    (deg): [%s]\n", joint_deg_str(theta_home));

homeErr_deg = abs(rad2deg(wrapToPi(theta_start - theta_home)));
if all(homeErr_deg <= homeTol_deg)
    fprintf("Already within +/-%.1f deg of HOME on all joints. Skipping HOME move.\n", homeTol_deg);
else
    disp("Moving to HOME slowly (quintic)...");
    move_slow_quintic_limited(hw, viewer, theta_start, theta_home, t_home_s, dt_home, maxDegPerSec_home);
end
disp("HOME reached.");
pause(0.5);

% =====================================================================
% STEP 2: PLANE CAPTURE
% =====================================================================
fprintf("\n=== STEP 2: PLANE CAPTURE ===\n");
disp("===========================================");
disp("PLANE CAPTURE: torque will turn OFF.");
disp("Move pen tip to 4 points on the board.");
disp("Press ENTER in MATLAB Command Window for each point.");
disp("Type q in MATLAB Command Window to abort.");
disp("===========================================");

hw.torqueOff();
pause(0.3);

P = zeros(3,4);
theta_cap = zeros(5,4);

for k = 1:4
    ok = wait_for_enter_or_q_cmd(viewer, hw, ...
        sprintf('Move to point %d, then press ENTER here in Command Window (or type q): ', k));
    if ~ok
        disp("Aborted during plane capture.");
        viewer.close();
        return
    end

    raw6 = hw.readMotors();
    report_servo_limit_violations(raw6, servo_calibration());

    theta_now = hw.readJoints();
    theta_cap(:,k) = theta_now;
    viewer.update(theta_now);

    T_now = FKinSpace(robot.M, robot.Slist, theta_now);
    P(:,k) = T_now(1:3,4);

    fprintf("P%d = [%.4f %.4f %.4f]^T   joints(deg)=[%s]\n", ...
        k, P(1,k), P(2,k), P(3,k), joint_deg_str(theta_now));
end

plane = plane_from_4pts(P, planePressOffset_m);
fprintf("Plane normal: [%.3f %.3f %.3f]\n", plane.z_hat);

[boardCenter, boardX, boardY] = board_frame_from_points(P, plane);
viewer.setBoard(boardCenter, boardX, boardY, boardSize_m, boardSize_m);

ok = confirm_cmd('Plane captured. Press ENTER to torque ON and continue, or type q to abort: ');
if ~ok
    disp("Aborted after plane capture.");
    viewer.close();
    return
end

disp("Torque ON...");
hw.torqueOn();
pause(0.4);

% =====================================================================
% STEP 3: GO TO WAYPOINT1
% =====================================================================
fprintf("\n=== STEP 3: GO TO WAYPOINT1 ===\n");
if use_waypoint1
    theta_now = hw.readJoints();
    viewer.update(theta_now);
    fprintf("  Current (deg):   [%s]\n", joint_deg_str(theta_now));
    fprintf("  Waypoint1 (deg): [%s]\n", joint_deg_str(theta_waypoint1));
    disp("Moving to permanent waypoint1...");
    move_slow_quintic_limited(hw, viewer, theta_now, theta_waypoint1, ...
        t_approach_s, dt_approach, maxDegPerSec_approach);
    disp("WAYPOINT1 reached.");
    pause(0.4);
end

% =====================================================================
% STEP 4: SKETCH CAPTURE
% =====================================================================
fprintf("\n=== STEP 4: SKETCH CAPTURE ===\n");
disp("Sketchpad capture...");
strokes = sketchpad_capture();
fprintf("Captured %d strokes.\n", numel(strokes));

if isempty(strokes)
    disp("No strokes captured. Exiting.");
    viewer.close();
    return;
end

% =====================================================================
% STEP 5: SCALE / CENTER
% =====================================================================
fprintf("\n=== STEP 5: SCALE & CENTER ===\n");
strokes2 = strokes_scale_center(strokes, desiredTextHeight_m);
fprintf("Scaled to %.1f inches (%.1f mm) height.\n", ...
    desiredTextHeight_in, desiredTextHeight_m*1000);

% =====================================================================
% STEP 6: STROKES -> TASK SPACE
% =====================================================================
fprintf("\n=== STEP 6: STROKES → TASK SPACE ===\n");
poses = strokes_to_taskspace(strokes2, plane, penUpHeight_m, boardBackoff_m);
fprintf("Generated %d task-space waypoints.\n", numel(poses));

if isempty(poses)
    disp("No task-space waypoints generated. Exiting.");
    viewer.close();
    return
end

p_first = poses(1).p(:);
dcap = vecnorm(P - p_first, 2, 1);
[~, idxSeed] = min(dcap);
theta_seed = theta_cap(:,idxSeed);

T_seed_fk = FKinSpace(robot.M, robot.Slist, theta_seed);
p_seed = T_seed_fk(1:3,4);

fprintf("  Seed capture idx: %d\n", idxSeed);
fprintf("  Seed tip:         [%.3f %.3f %.3f] m\n", p_seed(1), p_seed(2), p_seed(3));
fprintf("  First target:     [%.3f %.3f %.3f] m\n", p_first(1), p_first(2), p_first(3));
fprintf("  Seed distance:    %.3f m\n", norm(p_first - p_seed));

pathPts = zeros(3,numel(poses));
for i = 1:numel(poses)
    pathPts(:,i) = poses(i).p(:);
end
viewer.setPath(pathPts);
viewer.update(hw.readJoints());

% =====================================================================
% STEP 7: IK
% =====================================================================
fprintf("\n=== STEP 7: IK SOLVE ===\n");
disp("Solving position-only IK trajectory...");
fprintf("  Tolerance: %.1f mm,  Max iters: %d\n", ev*1000, maxIters);
fprintf("  IK seed (deg): [%s]\n", joint_deg_str(theta_seed));

thetaTraj = run_ik_trajectory(robot, poses, theta_seed, ev, maxIters);

fprintf("IK solved: %d waypoints.\n", size(thetaTraj,2));
for j = 1:5
    fprintf("  J%d range: [%+.1f, %+.1f] deg\n", j, ...
        rad2deg(min(thetaTraj(j,:))), rad2deg(max(thetaTraj(j,:))));
end

ok = confirm_cmd('Preview ready: path shown on 16x16 board. Press ENTER to execute, or type q to abort: ');
if ~ok
    disp("Execution aborted by user.");
    viewer.close();
    return
end

% =====================================================================
% STEP 8: EXECUTE DRAWING
% =====================================================================
fprintf("\n=== STEP 8: EXECUTE DRAWING ===\n");

theta_first_wp = thetaTraj(:,1);
disp("Moving to first drawing waypoint slowly...");
theta_pre = hw.readJoints();
viewer.update(theta_pre);
move_slow_quintic_limited(hw, viewer, theta_pre, theta_first_wp, ...
    t_approach_s, dt_approach, maxDegPerSec_approach);
pause(0.25);

fprintf("Drawing (%d waypoints at moderated speed)...\n", size(thetaTraj,2));
execute_trajectory_safe(hw, viewer, thetaTraj, dt_exec, maxDegPerSec_draw);

disp("Drawing complete. Returning HOME slowly...");
theta_end = hw.readJoints();
viewer.update(theta_end);
move_slow_quintic_limited(hw, viewer, theta_end, theta_home, ...
    t_home_s, dt_home, maxDegPerSec_home);

disp("=== DONE ===");
end

% #####################################################################
% LOCAL HELPERS
% #####################################################################

function move_slow_quintic_limited(hw, viewer, theta_start, theta_goal, duration_s, dt, maxDegPerSec)
theta_start = theta_start(:);
theta_goal  = theta_goal(:);

delta_base = wrapToPi(theta_goal(1) - theta_start(1));
goal_adjusted = theta_goal;
goal_adjusted(1) = theta_start(1) + delta_base;

maxStep = deg2rad(maxDegPerSec) * dt;
N_time  = max(3, ceil(duration_s / dt));
N_speed = max(3, ceil(max(abs(goal_adjusted - theta_start)) / maxStep));
N = max(N_time, N_speed);

for k = 1:N
    t = (k - 1) / (N - 1);
    s = 10*t^3 - 15*t^4 + 6*t^5;
    theta_cmd = (1 - s) * theta_start + s * goal_adjusted;
    hw.sendJoints(theta_cmd);
    viewer.update(theta_cmd);
    pause(dt);
end

hw.sendJoints(theta_goal);
viewer.update(theta_goal);
pause(0.15);
end

function execute_trajectory_safe(hw, viewer, thetaTraj, dt, maxDegPerSec)
theta_curr = hw.readJoints();
viewer.update(theta_curr);

maxStep = deg2rad(maxDegPerSec) * dt;

for i = 1:size(thetaTraj,2)
    theta_goal = thetaTraj(:,i);

    delta = theta_goal - theta_curr;
    delta(1) = wrapToPi(delta(1));

    nSteps = max(1, ceil(max(abs(delta)) / maxStep));

    for k = 1:nSteps
        alpha = k / nSteps;
        theta_cmd = theta_curr + alpha * delta;
        hw.sendJoints(theta_cmd);
        viewer.update(theta_cmd);
        pause(dt);
    end

    theta_curr = theta_goal;
end
end

function ok = wait_for_enter_or_q_cmd(viewer, hw, promptStr)
ok = true;

while true
    theta_now = hw.readJoints();
    viewer.update(theta_now);

    resp = input(promptStr, 's');
    if isempty(resp)
        ok = true;
        return
    end
    if strcmpi(strtrim(resp), 'q')
        ok = false;
        return
    end
end
end

function ok = confirm_cmd(promptStr)
resp = input(promptStr, 's');
ok = isempty(resp) || ~strcmpi(strtrim(resp), 'q');
end

function [center, xhat, yhat] = board_frame_from_points(P, plane)
center = mean(P,2);

xhat = P(:,2) - P(:,1);
zhat = plane.z_hat(:) / norm(plane.z_hat);
xhat = xhat - zhat * (zhat' * xhat);

if norm(xhat) < 1e-9
    xhat = P(:,4) - P(:,1);
    xhat = xhat - zhat * (zhat' * xhat);
end

xhat = xhat / norm(xhat);

yhat = cross(zhat, xhat);
yhat = yhat / norm(yhat);
end

function s = joint_deg_str(theta)
s = sprintf('%+.1f  ', rad2deg(theta(:)'));
s = strtrim(s);
end