function main_writeboard_demo()
% main_writeboard_demo
% End-to-end demo:
%   1) Torque ON, move to HOME from CURRENT measured pose
%   2) Torque OFF, capture 4 plane points (reading real joints each time)
%      then torque ON + return HOME from CURRENT measured pose
%   3) Capture strokes, scale/center
%   4) Convert strokes -> task-space waypoints on plane
%   5) Position-only IK trajectory
%   6) Execute, then return HOME

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

% ---- CALIB + HW ----
hw = robot_hw_rb150_calibrated_6motor("COM18", 1000000);
cal = servo_calibration(); %#ok<NASGU>

% ---- SETTINGS ----
theta_home = zeros(5,1);

% Plane
planePressOffset_m = 0.003;   % 3 mm into board
penUpHeight_m      = 0.015;   % 15 mm lift

% Motion timing
dt_home  = 0.05;
t_home_s = 3.0;

% Drawing scale
desiredTextHeight_in = 3.0;
desiredTextHeight_m  = desiredTextHeight_in * 0.0254;

% IK
ev = 1e-3;          % position tolerance (m)
maxIters = 120;     % more robust

% ---- STEP 1: Torque on + go HOME from current measured pose ----
disp("Torque ON...");
hw.torqueOn();

disp("Reading current joints...");
theta_start = hw.readJoints();

disp("Moving to HOME slowly...");
send_slow_shortest_base(hw, theta_start, theta_home, t_home_s, dt_home);

% ---- STEP 2: Plane capture with torque OFF ----
disp("===========================================");
disp("PLANE CAPTURE: torque will turn OFF.");
disp("Move pen tip to 4 points on the board.");
disp("Press ENTER for each point.");
disp("After point 4, press ENTER again to torque ON + return HOME slowly.");
disp("===========================================");

hw.torqueOff();

P = zeros(3,4);
for k = 1:4
    input(sprintf("Move to point %d, then press ENTER to capture...", k), "s");

    raw6 = hw.readMotors();                 % raw motor positions
    report_servo_limit_violations(raw6, servo_calibration());

    theta_now = hw.readJoints();            % 5x1 radians
    T_now = FKinSpace(robot.M, robot.Slist, theta_now);
    P(:,k) = T_now(1:3,4);

    fprintf("P%d = [%.4f %.4f %.4f]^T\n", k, P(1,k), P(2,k), P(3,k));
end

plane = plane_from_4pts(P, planePressOffset_m);

disp("Plane captured. Press ENTER to torque ON and return HOME...");
input("", "s");

hw.torqueOn();

% IMPORTANT: you moved the arm by hand, so re-read NOW, then return home
disp("Returning HOME (shortest base rotation)...");
theta_after = hw.readJoints();
send_slow_shortest_base(hw, theta_after, theta_home, t_home_s, dt_home);

% ---- STEP 3: Sketch capture ----
disp("Sketchpad capture...");
strokes = sketchpad_capture();

disp("Scaling / centering...");
strokes2 = strokes_scale_center(strokes, desiredTextHeight_m);

% ---- STEP 4: Strokes -> task-space poses ----
disp("Strokes -> task-space poses...");
poses = strokes_to_taskspace(strokes2, plane, penUpHeight_m);

% Debug: print first target and current FK at home
T_home = FKinSpace(robot.M, robot.Slist, theta_home);
p_home = T_home(1:3,4);
p1 = poses(1).p;
fprintf("DEBUG: p_home = [%.3f %.3f %.3f]\n", p_home(1), p_home(2), p_home(3));
fprintf("DEBUG: p_des1 = [%.3f %.3f %.3f]\n", p1(1), p1(2), p1(3));
fprintf("DEBUG: |p_des1-p_home| = %.3f m\n", norm(p1 - p_home));

% ---- STEP 5: IK ----
disp("Solving position-only IK trajectory...");
thetaTraj = run_ik_trajectory(robot, poses, hw.readJoints(), ev, maxIters);

% ---- STEP 6: Execute ----
disp("Executing trajectory...");
for i = 1:size(thetaTraj,2)
    hw.sendJoints(thetaTraj(:,i));
    pause(0.03);
end

disp("Return HOME...");
theta_end = hw.readJoints();
send_slow_shortest_base(hw, theta_end, theta_home, t_home_s, dt_home);

disp("Done.");
end

% ---------------- helpers ----------------
function send_slow_shortest_base(hw, th0, th1, T, dt)
th0 = th0(:); th1 = th1(:);
N = max(2, ceil(T/dt));

% make joint 1 (base) take the shortest route
d1 = wrapToPi(th1(1) - th0(1));
th1a = th0;
th1a(1) = th0(1) + d1;
th1a(2:5) = th1(2:5);

for k = 1:N
    a = (k-1)/(N-1);
    th = (1-a)*th0 + a*th1a;
    hw.sendJoints(th);
    pause(dt);
end
end
