function main_writeboard_demo()
% main_writeboard_demo
% End-to-end demo:
%   1) Torque ON, move to HOME from CURRENT measured pose
%   2) Torque OFF, capture 4 plane points (reading real joints each time)
%      then torque ON + return HOME from CURRENT measured pose
%   3) Capture strokes, scale/center
%   4) Convert strokes -> task-space waypoints on plane
%   5) Position-only IK trajectory (with automatic transit waypoints)
%   6) Execute, then return HOME
%
% FIXES (22-Feb-2026):
%   - Homing uses quintic (5th-order) time scaling for smooth accel/decel
%     instead of linear interp which depended on flush(s) not eating cmds.
%   - Return-to-home after plane capture works now (hw driver fix + quintic).
%   - Slow approach to first trajectory waypoint before drawing starts.
%   - IK uses transit waypoints (via fixed run_ik_trajectory.m).

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
hw = robot_hw_rb150_calibrated_6motor("COM18", 1000000);
cal = servo_calibration(); %#ok<NASGU>

% ---- SETTINGS ----
theta_home = zeros(5,1);

% Plane
planePressOffset_m = 0.003;   % 3 mm into board
penUpHeight_m      = 0.015;   % 15 mm lift

% Motion timing
dt_cmd   = 0.04;              % command interval during slow moves (25 Hz)
t_home_s = 4.0;               % seconds for slow home motion
dt_exec  = 0.03;              % execution interval during drawing (33 Hz)

% Drawing scale
desiredTextHeight_in = 3.0;
desiredTextHeight_m  = desiredTextHeight_in * 0.0254;

% IK
ev = 1e-3;          % position tolerance (m)
maxIters = 200;     % generous — standalone solver is robust

% =====================================================================
%  STEP 1: Torque on + go HOME from current measured pose
% =====================================================================
fprintf("\n=== STEP 1: GO HOME ===\n");
disp("Torque ON...");
hw.torqueOn();
pause(0.3);

disp("Reading current joints...");
theta_start = hw.readJoints();
fprintf("  Current (deg): [%s]\n", joint_deg_str(theta_start));
fprintf("  Home    (deg): [%s]\n", joint_deg_str(theta_home));

disp("Moving to HOME slowly (quintic)...");
move_slow_quintic(hw, theta_start, theta_home, t_home_s, dt_cmd);
disp("HOME reached.");
pause(0.5);

% =====================================================================
%  STEP 2: Plane capture with torque OFF
% =====================================================================
fprintf("\n=== STEP 2: PLANE CAPTURE ===\n");
disp("===========================================");
disp("PLANE CAPTURE: torque will turn OFF.");
disp("Move pen tip to 4 points on the board.");
disp("Press ENTER for each point.");
disp("After point 4, press ENTER again to torque ON + return HOME.");
disp("===========================================");

hw.torqueOff();
pause(0.3);

P = zeros(3,4);
for k = 1:4
    input(sprintf("Move to point %d, then press ENTER to capture...", k), "s");

    raw6 = hw.readMotors();
    report_servo_limit_violations(raw6, servo_calibration());

    theta_now = hw.readJoints();
    T_now = FKinSpace(robot.M, robot.Slist, theta_now);
    P(:,k) = T_now(1:3,4);

    fprintf("P%d = [%.4f %.4f %.4f]^T   joints(deg)=[%s]\n", ...
        k, P(1,k), P(2,k), P(3,k), joint_deg_str(theta_now));
end

plane = plane_from_4pts(P, planePressOffset_m);
fprintf("Plane normal: [%.3f %.3f %.3f]\n", plane.z_hat);

disp("Plane captured. Press ENTER to torque ON and return HOME...");
input("", "s");

% Torque ON + slow return home
disp("Torque ON...");
hw.torqueOn();
pause(0.3);

disp("Returning HOME slowly (quintic)...");
theta_after = hw.readJoints();
fprintf("  Current (deg): [%s]\n", joint_deg_str(theta_after));
move_slow_quintic(hw, theta_after, theta_home, t_home_s, dt_cmd);
disp("HOME reached.");
pause(0.5);

% =====================================================================
%  STEP 3: Sketch capture
% =====================================================================
fprintf("\n=== STEP 3: SKETCH CAPTURE ===\n");
disp("Sketchpad capture...");
strokes = sketchpad_capture();
fprintf("Captured %d strokes.\n", numel(strokes));

if isempty(strokes)
    disp("No strokes captured. Exiting.");
    return;
end

% =====================================================================
%  STEP 4: Scale / center
% =====================================================================
fprintf("\n=== STEP 4: SCALE & CENTER ===\n");
strokes2 = strokes_scale_center(strokes, desiredTextHeight_m);
fprintf("Scaled to %.1f inches (%.1f mm) height.\n", ...
    desiredTextHeight_in, desiredTextHeight_m*1000);

% =====================================================================
%  STEP 5: Strokes -> task-space poses
% =====================================================================
fprintf("\n=== STEP 5: STROKES → TASK SPACE ===\n");
poses = strokes_to_taskspace(strokes2, plane, penUpHeight_m);
fprintf("Generated %d task-space waypoints.\n", numel(poses));

% Diagnostics
T_home_fk = FKinSpace(robot.M, robot.Slist, theta_home);
p_home = T_home_fk(1:3,4);
p1 = poses(1).p;
fprintf("  Home tip:     [%.3f %.3f %.3f] m\n", p_home(1), p_home(2), p_home(3));
fprintf("  First target: [%.3f %.3f %.3f] m\n", p1(1), p1(2), p1(3));
fprintf("  Distance:     %.3f m\n", norm(p1 - p_home));

% =====================================================================
%  STEP 6: IK
% =====================================================================
fprintf("\n=== STEP 6: IK SOLVE ===\n");
disp("Solving position-only IK trajectory...");
fprintf("  Tolerance: %.1f mm,  Max iters: %d\n", ev*1000, maxIters);

theta_seed = hw.readJoints();
thetaTraj = run_ik_trajectory(robot, poses, theta_seed, ev, maxIters);

fprintf("IK solved: %d waypoints.\n", size(thetaTraj,2));

% Quick joint range check
for j = 1:5
    fprintf("  J%d range: [%+.1f, %+.1f] deg\n", j, ...
        rad2deg(min(thetaTraj(j,:))), rad2deg(max(thetaTraj(j,:))));
end

% =====================================================================
%  STEP 7: Execute trajectory → return HOME
% =====================================================================
fprintf("\n=== STEP 7: EXECUTE ===\n");

% Slowly approach the first waypoint before starting
theta_first_wp = thetaTraj(:,1);
disp("Moving to first waypoint slowly...");
theta_pre = hw.readJoints();
move_slow_quintic(hw, theta_pre, theta_first_wp, 2.0, dt_cmd);
pause(0.3);

% Execute the drawing trajectory
fprintf("Drawing (%d waypoints at %.0f Hz)...\n", size(thetaTraj,2), 1/dt_exec);
for i = 1:size(thetaTraj,2)
    hw.sendJoints(thetaTraj(:,i));
    pause(dt_exec);
end

% Return home
disp("Drawing complete. Returning HOME slowly...");
theta_end = hw.readJoints();
move_slow_quintic(hw, theta_end, theta_home, t_home_s, dt_cmd);

disp("=== DONE ===");
end


% #####################################################################
%  LOCAL HELPERS
% #####################################################################

function move_slow_quintic(hw, theta_start, theta_goal, duration_s, dt)
% Smoothly interpolate from theta_start to theta_goal using quintic
% (5th-order) time scaling.  s(t) = 10t^3 - 15t^4 + 6t^5 gives zero
% velocity AND zero acceleration at both endpoints — no jerk.
%
% Joint 1 (base) takes the shortest angular path via wrapToPi.

    theta_start = theta_start(:);
    theta_goal  = theta_goal(:);

    % Shortest path for base rotation
    delta_base     = wrapToPi(theta_goal(1) - theta_start(1));
    goal_adjusted  = theta_goal;
    goal_adjusted(1) = theta_start(1) + delta_base;

    N = max(3, ceil(duration_s / dt));

    for k = 1:N
        t = (k - 1) / (N - 1);    % normalised time [0, 1]

        % Quintic time scaling: zero vel + zero accel at endpoints
        s = 10*t^3 - 15*t^4 + 6*t^5;

        theta_cmd = (1 - s) * theta_start + s * goal_adjusted;
        hw.sendJoints(theta_cmd);
        pause(dt);
    end

    % Final: send exact goal
    hw.sendJoints(theta_goal);
    pause(0.1);
end


function s = joint_deg_str(theta)
    s = sprintf('%+.1f  ', rad2deg(theta(:)'));
    s = strtrim(s);
end
