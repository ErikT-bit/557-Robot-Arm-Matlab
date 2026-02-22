function main_writeboard_demo()
% Workflow:
% 1) Home pose (all zeros)
% 2) Capture 4 points on the writing surface
% 3) Capture strokes (mouse drawing)
% 4) Scale/center to 2–4 inches height
% 5) Convert to task-space poses on the plane
% 6) Solve IK along the trajectory (Modern Robotics)
% 7) Execute (RB150 motor-1 only test) and return home
%
% NOTE (Step 3): Hardware layer currently drives ONLY Dynamixel ID=1 using
% byte protocol (goalPos 0..1023). Full 5-joint motion comes in Step 4.

clc; close all;

% ---- PATHS (NO ABSOLUTE PATHS) ----
thisFile = mfilename('fullpath');
scriptsDir = fileparts(thisFile);              % .../MATLAB/scripts
matlabDir  = fileparts(scriptsDir);            % .../MATLAB

mrDir    = fullfile(matlabDir, "mr");
urdfPath = fullfile(matlabDir, "simple_robot_v2.urdf");

addpath(genpath(mrDir));
addpath(scriptsDir);

% ---- ROBOT SETTINGS (MATCH YOUR URDF) ----
baseLink = "FixedBase";
tipLink  = "PenTipLink";
jointNames5 = ["RotatingBaseJoint","ShoulderJoint","ElbowJoint","WristJoint","PenJoint"];

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


% ---- RB150 hardware (Step 3: drive ONE motor using byte protocol) ----
% This hardware layer ONLY supports hw.sendGoal(goalPos) for motor ID=1
jointMap = []; % we’ll tune dir signs later
hw = robot_hw_rb150_byte_5motor("COM18", 1000000, jointMap);
hw.torqueOn();

% ---- "Home" for Step 3 (motor 1 only) ----
mid  = 512;
dir1 = +1;   % flip to -1 if direction reversed

% Track the commanded joints in software (since we can't read 5 joints yet)
thetalist_cmd = robot.home;

% Send motor-1 to center position as a safe start
hw.sendGoal(mid);
fprintf("Sent Motor-1 to mid (512). Full-arm home will be Step 4.\n");

% 2) Capture plane points
disp("Plane calibration:");
disp("Manually move the robot tip to 4 points on the board.");
disp("Each time you're ready to record a point, press ENTER in the command window.");
disp("NOTE: Step 3 cannot read all joints from hardware yet, so plane capture will");
disp("use the current commanded joint state. In Step 4 we will read real joints.");

P = zeros(3,4);
for k = 1:4
    input(sprintf("Move to point %d then press ENTER to capture...", k), "s");

    % Using last commanded thetalist (software state)
  thetalist_now = hw.readJoints();
  T_now = FKinSpace(robot.M, robot.Slist, thetalist_now);
    P(:,k) = T_now(1:3,4);

    fprintf("Captured P%d = [%.4f %.4f %.4f]^T (from commanded joints)\n", ...
        k, P(1,k), P(2,k), P(3,k));
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
theta0 = robot.home;
thetaTraj = run_ik_trajectory(robot, poses, theta0, ikTolW, ikTolV, maxIKIters);

% 7) Execute (Motor 1 only)
disp("Executing trajectory (Step 3: Motor-1 only) ...");

for i = 1:size(thetaTraj,2)
    thetalist_cmd = thetaTraj(:,i);              % update software state

    theta1 = thetaTraj(1,i);                     % radians
    gp1 = theta_to_goalpos_ax(theta1, mid, dir1); % 0..1023

    hw.sendGoal(gp1);                            % sends 2 bytes, reads 2 bytes back
    pause(dt);
end

disp("Returning Motor-1 to mid...");
hw.sendGoal(mid);

disp("Done (Step 3 complete).");
end
