function main_writeboard_demo()
% Workflow:
% 1) Home pose (all zeros)
% 2) Capture 4 points on the writing surface
% 3) Capture strokes (mouse drawing)
% 4) Scale/center to 2–4 inches height
% 5) Convert to task-space poses on the plane
% 6) Solve IK along the trajectory (Modern Robotics)
% 7) Execute (stub) and return home

clc; close all;

% ---- PATHS (NO ABSOLUTE PATHS) ----
thisFile = mfilename('fullpath');
scriptsDir = fileparts(thisFile);              % .../MATLAB/scripts
matlabDir  = fileparts(scriptsDir);            % .../MATLAB
repoRoot   = fileparts(matlabDir);             % repo root

mrDir   = fullfile(matlabDir, "mr");
urdfPath = fullfile(matlabDir, "simple_robot_v2.urdf");

addpath(genpath(mrDir));
addpath(scriptsDir);

% ---- ROBOT SETTINGS ----
baseLink = "base_link";
tipLink  = "Pen_tip";

% If your URDF uses different joint names, change these to match URDF.
% (The order defines your thetalist order used everywhere.)
jointNames5 = ["Motor1_joint","Motor2_L","Motor4_elb","Motor5_wr","Joint_EE"];

% ---- WRITING SETTINGS ----
desiredTextHeight_in = 3.0;              % between 2–4 inches
desiredTextHeight_m  = desiredTextHeight_in * 0.0254;

planePressOffset_m   = 0.003;            % 3 mm "behind" surface for pressure
penUpHeight_m        = 0.015;            % lift pen 15 mm off the board

dt = 0.03;
ikTolW = 1e-3;
ikTolV = 1e-3;
maxIKIters = 40;

% ---- Load robot model (PoE) from URDF ----
robot = robot_model_from_urdf(urdfPath, baseLink, tipLink, jointNames5);

fprintf("Loaded URDF:\n  %s\n", urdfPath);
disp("Home joint state (rad):"); disp(robot.home(:).');

% ---- Hardware interface (stub for now) ----
hw = robot_hw_stub(robot);

% 1) Home
thetalist_home = robot.home;
hw.sendJoints(thetalist_home);
fprintf("Sent HOME (all zeros).\n");

% 2) Capture plane points
disp("Plane calibration:");
disp("Manually move the robot tip to 4 points on the board.");
disp("Each time you're ready to record a point, press ENTER in the command window.");

P = zeros(3,4);
for k = 1:4
    input(sprintf("Move to point %d then press ENTER to capture...", k), "s");
    thetalist_now = hw.readJoints();  % replace with real joint read later
    T_now = FKinSpace(robot.M, robot.Slist, thetalist_now);
    P(:,k) = T_now(1:3,4);
    fprintf("Captured P%d = [%.4f %.4f %.4f]^T\n", k, P(1,k), P(2,k), P(3,k));
end

plane = plane_from_4pts(P, planePressOffset_m);

% 3) Sketch capture
disp("Sketchpad capture (mouse):");
strokes = sketchpad_capture();

% 4) Scale/center to desired height
strokes2 = strokes_scale_center(strokes, desiredTextHeight_m);

% 5) Strokes -> task-space SE(3)
poses = strokes_to_taskspace(strokes2, plane, penUpHeight_m);

% 6) IK along trajectory
disp("Solving IK trajectory...");
theta0 = thetalist_home;
thetaTraj = run_ik_trajectory(robot, poses, theta0, ikTolW, ikTolV, maxIKIters);

% 7) Execute and return home
disp("Executing trajectory (stubbed send)...");
for i = 1:size(thetaTraj,2)
    hw.sendJoints(thetaTraj(:,i));
    pause(dt);
end

disp("Returning HOME...");
hw.sendJoints(thetalist_home);
disp("Done.");
end
