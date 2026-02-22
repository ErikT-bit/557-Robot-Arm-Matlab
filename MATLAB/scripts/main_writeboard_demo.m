function main_writeboard_demo()
clc; close all;

% ---- PATHS ----
thisFile   = mfilename('fullpath');
scriptsDir = fileparts(thisFile);
matlabDir  = fileparts(scriptsDir);

addpath(genpath(fullfile(matlabDir,"mr")));
addpath(scriptsDir);

urdfPath = fullfile(matlabDir, "simple_robot_v2.urdf");

% ---- URDF names ----
baseLink    = "FixedBase";
tipLink     = "PenTipLink";
jointNames5 = ["RotatingBaseJoint","ShoulderJoint","ElbowJoint","WristJoint","PenJoint"];

% ---- Robot model ----
robot = robot_model_from_urdf(urdfPath, baseLink, tipLink, jointNames5);

% ---- MoveIt limits + home ----
limits = robot_joint_limits();
theta_home = zeros(5,1);

% ---- Calibration ----
cal = servo_calibration();

% ---- Writing params ----
desiredTextHeight_in = 3.0;
desiredTextHeight_m  = desiredTextHeight_in * 0.0254;

planePressOffset_m = 0.003;
penUpHeight_m      = 0.020;   % a bit higher to avoid accidental marks
approachLift_m     = 0.030;   % lift before lateral travel

% ---- IK params ----
ev = 2e-3;
maxIters = 120;

% ---- Timing ----
dt_exec  = 0.03;
dt_home  = 0.05;
t_home_s = 6.0;

% ---- Hardware ----
hw = robot_hw_rb150_calibrated_6motor("COM18", 1000000);

disp("Torque ON...");
hw.torqueOn();

% Move to home slowly from current (shortest base rotation)
theta_start = hw.readJoints();
disp("Moving to HOME slowly...");
send_slow_shortest_base(hw, theta_start, theta_home, t_home_s, dt_home, limits);

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

    raw6 = hw.readMotors();
    report_servo_limit_violations(raw6, cal);

    theta_k = servo_to_moveit_rad(raw6, cal);   % actual estimated MoveIt joints
    % DO NOT clamp theta_k for FK; we want real pose estimate here.
    T_k = FKinSpace(robot.M, robot.Slist, theta_k);
    P(:,k) = T_k(1:3,4);

    fprintf("P%d = [%.4f %.4f %.4f]^T\n", k, P(1,k), P(2,k), P(3,k));
end

plane = plane_from_4pts(P, planePressOffset_m);

input("Plane captured. Press ENTER to torque ON and return HOME...", "s");
hw.torqueOn();
pause(0.15);

theta_now = hw.readJoints();
disp("Returning HOME (shortest base rotation)...");
send_slow_shortest_base(hw, theta_now, theta_home, t_home_s, dt_home, limits);

% =========================
% SKETCH
% =========================
disp("Sketchpad capture...");
strokes = sketchpad_capture();

disp("Scaling / centering...");
strokes2 = strokes_scale_center(strokes, desiredTextHeight_m);

disp("Strokes -> task-space poses...");
poses = strokes_to_taskspace(strokes2, plane, penUpHeight_m);

% Insert APPROACH sequence (lift at home tip -> travel above first point -> descend)
poses = prepend_approach(robot, plane, theta_home, poses, approachLift_m);

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
send_slow_shortest_base(hw, hw.readJoints(), theta_home, t_home_s, dt_home, limits);

disp("Done.");
end

% ---------------- helpers ----------------

function send_slow_shortest_base(hw, th0, th1, Tsec, dt, limits)
% Smooth interpolation, but base joint uses shortest angular difference.
N = max(2, round(Tsec/dt));
th0 = th0(:); th1 = th1(:);

% shortest-path adjust for joint 1 (base)
d1 = wrapToPi(th1(1) - th0(1));
th1_adj = th1;
th1_adj(1) = th0(1) + d1;

for i = 1:N
    s = (i-1)/(N-1);
    s = s*s*(3-2*s);
    th = (1-s)*th0 + s*th1_adj;
    [th, ~] = clamp_to_limits(th, limits);  % clamp COMMANDS only
    hw.sendJoints(th);
    pause(dt);
end
end

function a = wrapToPi(a)
a = mod(a + pi, 2*pi) - pi;
end

function poses2 = prepend_approach(robot, plane, theta_home, poses, lift_m)
% Add: lift at home tip -> travel above first pose -> go to first pose
if isempty(poses)
    poses2 = poses; return;
end

T_home = FKinSpace(robot.M, robot.Slist, theta_home);
p_home = T_home(1:3,4);

% tool orientation consistent with plane
z_tool = -plane.z_hat;
x_tool = plane.x_hat;
y_tool = cross(z_tool, x_tool); y_tool = y_tool / norm(y_tool);
R = [x_tool, y_tool, z_tool];

% First pose target
T1 = poses(1).T;
p1 = T1(1:3,4);

% 1) lift at home location along +plane.z_hat
Tlift = eye(4); Tlift(1:3,1:3) = R;
Tlift(1:3,4) = p_home + plane.z_hat*lift_m;

% 2) travel above first point
Ttravel = eye(4); Ttravel(1:3,1:3) = R;
Ttravel(1:3,4) = p1 + plane.z_hat*lift_m;

poses2 = [struct('T',Tlift); struct('T',Ttravel); poses(:)];
end
