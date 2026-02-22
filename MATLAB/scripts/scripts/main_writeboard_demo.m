function main_writeboard_demo()
% main_writeboard_demo
% Step 4 version: real 5-motor RB150 bridge with torque-off plane capture.
%
% Workflow:
% 0) Connect hardware (RB150 5-motor byte protocol)
% 1) Home pose
% 2) Capture 4 points on writing plane (TORQUE OFF to manually move)
%    - After point 4, press ENTER again -> TORQUE ON and slow return home
% 3) Sketch capture (mouse)
% 4) Scale/center to 2–4 inches height
% 5) Convert strokes -> task-space poses on the plane
% 6) IK trajectory (Modern Robotics)
% 7) Execute trajectory on hardware and return home

clc; close all;

% ---- PATHS (NO ABSOLUTE PATHS) ----
thisFile    = mfilename('fullpath');
scriptsDir  = fileparts(thisFile);      % .../MATLAB/scripts
matlabDir   = fileparts(scriptsDir);    % .../MATLAB

mrDir    = fullfile(matlabDir, "mr");
urdfPath = fullfile(matlabDir, "simple_robot_v2.urdf");

addpath(genpath(mrDir));
addpath(scriptsDir);

% ---- ROBOT SETTINGS (MATCH YOUR URDF) ----
baseLink    = "FixedBase";
tipLink     = "PenTipLink";
jointNames5 = ["RotatingBaseJoint","ShoulderJoint","ElbowJoint","WristJoint","PenJoint"];

% ---- SERIAL / HARDWARE SETTINGS ----
port = "COM18";
baud = 1000000;

% ---- WRITING SETTINGS ----
desiredTextHeight_in = 3.0;                  % between 2–4 inches
desiredTextHeight_m  = desiredTextHeight_in * 0.0254;

planePressOffset_m   = 0.003;                % 3 mm "behind" surface for pressure
penUpHeight_m        = 0.015;                % lift pen 15 mm off board

% timing
dt_exec   = 0.03;                            % execution dt
dt_home   = 0.05;                            % slower return-to-home dt
t_home_s  = 5.0;                             % seconds for slow return to home

% IK settings
ikTolW = 1e-3;
ikTolV = 1e-3;
maxIKIters = 40; %#ok<NASGU>  % kept for interface compatibility

% ---- Load robot model (PoE) from URDF ----
robot = robot_model_from_urdf(urdfPath, baseLink, tipLink, jointNames5);
fprintf("Loaded URDF:\n  %s\n", urdfPath);
disp("Home joint state (rad):"); disp(robot.home(:).');

% ---- Hardware interface (RB150 5-motor) ----
jointMap = []; % defaults: mid=512, dir=+1, scale=AX counts/rad
hw = robot_hw_rb150_byte_5motor(port, baud, jointMap);

% -------------------------------
% 1) Home pose (TORQUE ON)
% -------------------------------
disp("Torque ON and sending HOME...");
hw.torqueOn();
thetalist_home = robot.home(:);
hw.sendJoints(thetalist_home);
pause(0.25);

% -------------------------------
% 2) Plane capture (TORQUE OFF)
% -------------------------------
disp("===========================================");
disp("PLANE CALIBRATION MODE");
disp("Torque will turn OFF so you can move the arm by hand.");
disp("Move the pen tip to 4 points on the board.");
disp("Each time you're ready, press ENTER in the Command Window to capture.");
disp("===========================================");

hw.torqueOff();
pause(0.25);

P = zeros(3,4);
for k = 1:4
    input(sprintf("Move to point %d, then press ENTER to capture...", k), "s");

    % Read actual joints from Dynamixel present positions
    thetalist_now = hw.readJoints();    % 5x1 rad
    T_now = FKinSpace(robot.M, robot.Slist, thetalist_now);

    P(:,k) = T_now(1:3,4);
    fprintf("Captured P%d = [%.4f %.4f %.4f]^T\n", k, P(1,k), P(2,k), P(3,k));
end

% Build plane
plane = plane_from_4pts(P, planePressOffset_m);

disp("Plane captured. Press ENTER to re-enable torque and return HOME slowly...");
input("", "s");

% -------------------------------
% Torque on + slow return home
% -------------------------------
disp("Torque ON. Returning HOME slowly...");
hw.torqueOn();
pause(0.15);

% Start from current measured joint state (still at point 4)
theta_start = hw.readJoints();
theta_goal  = thetalist_home;

Nhome = max(2, round(t_home_s / dt_home));
for i = 1:Nhome
    s = (i-1)/(Nhome-1);
    % smoothstep for gentle start/stop
    s2 = s*s*(3 - 2*s);

    theta_cmd = (1 - s2)*theta_start + s2*theta_goal;
    hw.sendJoints(theta_cmd);
    pause(dt_home);
end
disp("Arrived HOME.");

% -------------------------------
% 3) Sketch capture (mouse)
% -------------------------------
disp("===========================================");
disp("SKETCHPAD CAPTURE");
disp("Draw what you want. Release click = pen up.");
disp("Close/finish the sketchpad window when done.");
disp("===========================================");
strokes = sketchpad_capture();

% -------------------------------
% 4) Scale/center to desired height
% -------------------------------
strokes2 = strokes_scale_center(strokes, desiredTextHeight_m);

% -------------------------------
% 5) Strokes -> task-space SE(3) poses
% -------------------------------
poses = strokes_to_taskspace(strokes2, plane, penUpHeight_m);

% -------------------------------
% 6) IK along trajectory
% -------------------------------
disp("Solving IK trajectory...");
theta0 = thetalist_home;
thetaTraj = run_ik_trajectory(robot, poses, theta0, ikTolW, ikTolV, maxIKIters);

% -------------------------------
% 7) Execute trajectory and return home
% -------------------------------
disp("Executing trajectory on hardware...");
for i = 1:size(thetaTraj,2)
    hw.sendJoints(thetaTraj(:,i));
    pause(dt_exec);
end

disp("Returning HOME...");
hw.sendJoints(thetalist_home);

disp("Done.");
end