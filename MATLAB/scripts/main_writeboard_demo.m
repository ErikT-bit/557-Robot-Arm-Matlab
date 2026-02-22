function main_writeboard_demo()
% main_writeboard_demo (Step 4)
% - Uses calibrated 6-motor hardware mapping (servo_calibration + MoveIt mapping)
% - Torque OFF during plane capture
% - After capture: ENTER -> torque ON and slow return HOME
% - Sketch -> scale -> plane poses -> position-only IK -> execute

clc; close all;

% ---- PATHS (repo-relative) ----
thisFile   = mfilename('fullpath');
scriptsDir = fileparts(thisFile);
matlabDir  = fileparts(scriptsDir);

addpath(genpath(fullfile(matlabDir,"mr")));
addpath(scriptsDir);

urdfPath = fullfile(matlabDir, "simple_robot_v2.urdf");

% ---- URDF names ----
baseLink    = "FixedBase";
tipLink     = "PenTipLink";
jointNames5 = ["RotatingBaseJoint","ShoulderJoint","ElbowJoint","WristJoint","PenJoint"]; %#ok<NASGU>

% ---- Robot model ----
robot = robot_model_from_urdf(urdfPath, baseLink, tipLink, ...
    ["RotatingBaseJoint","ShoulderJoint","ElbowJoint","WristJoint","PenJoint"]);

% ---- Limits + home (MoveIt convention) ----
limits = robot_joint_limits();
theta_home = zeros(5,1);

% ---- Writing params ----
desiredTextHeight_in = 3.0;
desiredTextHeight_m  = desiredTextHeight_in * 0.0254;

planePressOffset_m = 0.003;
penUpHeight_m      = 0.015;

% ---- IK params ----
ev = 2e-3;
maxIters = 80;

% ---- Timing ----
dt_exec  = 0.03;
dt_home  = 0.05;
t_home_s = 6.0;

% ---- Hardware ----
hw = robot_hw_rb150_calibrated_6motor("COM18", 1000000);

disp("Torque ON...");
hw.torqueOn();

% Move to home slowly from current
theta_start = hw.readJoints();
[theta_start, ~] = clamp_to_limits(theta_start, limits);
[theta_home,  ~] = clamp_to_limits(theta_home,  limits);

disp("Moving to HOME slowly...");
send_slow(hw, theta_start, theta_home, t_home_s, dt_home, limits);

% =========================
% PLANE CAPTURE
% =========================
disp("===========================================");
disp("PLANE CAPTURE: torque will turn OFF.");
disp("Move pen tip to 4 points on the board.");
disp("Press ENTER for each point.");
disp("After point 4, press ENTER again to torque ON + return HOME slowly.");
disp("===========================================");

hw.torqueOff();
pause(0.25);

P = zeros(3,4);
for k = 1:4
    input(sprintf("Move to point %d, then press ENTER to capture...", k), "s");

    theta_k = hw.readJoints();
    [theta_k, violated] = clamp_to_limits(theta_k, limits);
    if violated
        warning("During plane capture, joint limits were exceeded; clamped estimate.");
    end

    T_k = FKinSpace(robot.M, robot.Slist, theta_k);
    P(:,k) = T_k(1:3,4);

    fprintf("P%d = [%.4f %.4f %.4f]^T\n", k, P(1,k), P(2,k), P(3,k));
end

plane = plane_from_4pts(P, planePressOffset_m);

input("Plane captured. Press ENTER to torque ON and return HOME...", "s");

hw.torqueOn();
pause(0.15);

theta_now = hw.readJoints();
send_slow(hw, theta_now, theta_home, t_home_s, dt_home, limits);

% =========================
% SKETCH
% =========================
disp("Sketchpad capture...");
strokes = sketchpad_capture();

disp("Scaling / centering...");
strokes2 = strokes_scale_center(strokes, desiredTextHeight_m);

disp("Strokes -> task-space poses...");
poses = strokes_to_taskspace(strokes2, plane, penUpHeight_m);

% =========================
% IK
% =========================
disp("Solving position-only IK trajectory...");
thetaTraj = run_ik_trajectory(robot, poses, theta_home, limits, ev, maxIters);

% =========================
% EXECUTE
% =========================
disp("Executing trajectory...");
for k = 1:size(thetaTraj,2)
    hw.sendJoints(thetaTraj(:,k));
    pause(dt_exec);
end

disp("Returning HOME...");
send_slow(hw, hw.readJoints(), theta_home, t_home_s, dt_home, limits);

disp("Done.");
end

function send_slow(hw, th0, th1, Tsec, dt, limits)
N = max(2, round(Tsec/dt));
th0 = th0(:); th1 = th1(:);

for i = 1:N
    s = (i-1)/(N-1);
    s = s*s*(3-2*s); % smoothstep
    th = (1-s)*th0 + s*th1;
    [th, ~] = clamp_to_limits(th, limits);
    hw.sendJoints(th);
    pause(dt);
end
end
