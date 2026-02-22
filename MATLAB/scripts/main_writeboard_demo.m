function main_writeboard_demo()
% Step 4: 6 motors (IDs 1..6), 5 DOF with motor2 mirroring motor3.
% Automatic plane capture:
%   - torque OFF during capture
%   - capture 4 points (ENTER each)
%   - ENTER again -> torque ON + slow return HOME
% Then: sketch -> scale -> taskspace -> position-only IK -> execute.

clc; close all;

% ---- PATHS ----
thisFile   = mfilename('fullpath');
scriptsDir = fileparts(thisFile);
matlabDir  = fileparts(scriptsDir);

addpath(genpath(fullfile(matlabDir,"mr")));
addpath(scriptsDir);

urdfPath = fullfile(matlabDir, "simple_robot_v2.urdf");

% ---- URDF NAMES ----
baseLink    = "FixedBase";
tipLink     = "PenTipLink";
jointNames5 = ["RotatingBaseJoint","ShoulderJoint","ElbowJoint","WristJoint","PenJoint"];

% ---- Comms ----
port = "COM18";
baud = 1000000;

% ---- Writing params ----
desiredTextHeight_in = 3.0;
desiredTextHeight_m  = desiredTextHeight_in * 0.0254;

planePressOffset_m   = 0.003;
penUpHeight_m        = 0.015;

% ---- Motion timing ----
dt_exec   = 0.03;
dt_home   = 0.05;
t_home_s  = 6.0;  % slow return duration

% ---- IK params ----
ev = 2e-3;        % 2 mm position tolerance
maxIters = 80;

% ---- Load robot model ----
robot = robot_model_from_urdf(urdfPath, baseLink, tipLink, jointNames5);
fprintf("Loaded URDF:\n  %s\n", urdfPath);

% ---- Joint limits (from URDF) ----
limits = urdf_joint_limits(urdfPath, jointNames5);  % nx2
robot.limits = limits;

% ---- Define a SAFE home inside limits ----
% Home MUST be within limits. Use midpoints as safe default.
theta_home = mean(limits,2);

% ---- Hardware (6 motors w/ mirror constraint) ----
hw = robot_hw_rb150_byte_6motor(port, baud, []);

% ---- Torque ON, go HOME smoothly (from wherever it is) ----
disp("Torque ON...");
hw.torqueOn();

theta_start = hw.readJoints();
theta_home  = clamp_to_limits(theta_home, robot.limits);

disp("Moving to HOME slowly...");
send_slow(hw, theta_start, theta_home, t_home_s, dt_home);

% =========================
% STEP 2: PLANE CAPTURE
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
theta_pts = zeros(5,4);

for k = 1:4
    input(sprintf("Move to point %d, then press ENTER to capture...", k), "s");

    theta_k = hw.readJoints();
    theta_k = clamp_to_limits(theta_k, robot.limits);

    T_k = FKinSpace(robot.M, robot.Slist, theta_k);

    P(:,k) = T_k(1:3,4);
    theta_pts(:,k) = theta_k;

    fprintf("P%d = [%.4f %.4f %.4f]^T\n", k, P(1,k), P(2,k), P(3,k));
end

plane = plane_fit_ls(P, planePressOffset_m);

input("Plane captured. Press ENTER to torque ON and return HOME...", "s");

hw.torqueOn();
pause(0.15);
theta_now = hw.readJoints();
send_slow(hw, theta_now, theta_home, t_home_s, dt_home);

% =========================
% STEP 3: SKETCH
% =========================
disp("Sketchpad capture...");
strokes = sketchpad_capture();

% scale/center
strokes2 = strokes_scale_center(strokes, desiredTextHeight_m);

% strokes -> taskspace (we will add pen transitions here)
poses = strokes_to_taskspace(strokes2, plane, penUpHeight_m);
poses = add_pen_transitions(poses, 0.005, 12); % lift=5mm, 12 interpolation steps

% =========================
% STEP 4: POSITION-ONLY IK TRAJECTORY
% =========================
disp("Solving position-only IK trajectory...");
thetaTraj = zeros(5, numel(poses));
theta = theta_home;

for k = 1:numel(poses)
    Tsd = poses(k).T;
    p_des = Tsd(1:3,4);

    [theta, ok] = ik_position_only_space(robot.Slist, robot.M, p_des, theta, ev, maxIters);
    if ~ok
        error("Position-only IK failed at waypoint %d/%d", k, numel(poses));
    end

    theta = clamp_to_limits(theta, robot.limits);
    thetaTraj(:,k) = theta;
end

% =========================
% STEP 5: EXECUTE
% =========================
disp("Executing trajectory...");
for k = 1:size(thetaTraj,2)
    hw.sendJoints(thetaTraj(:,k));
    pause(dt_exec);
end

disp("Returning HOME...");
send_slow(hw, hw.readJoints(), theta_home, t_home_s, dt_home);

disp("Done.");
end

% -------- helpers --------

function send_slow(hw, th0, th1, Tsec, dt)
N = max(2, round(Tsec/dt));
th0 = th0(:); th1 = th1(:);
for i = 1:N
    s = (i-1)/(N-1);
    s = s*s*(3-2*s); % smoothstep
    th = (1-s)*th0 + s*th1;
    hw.sendJoints(th);
    pause(dt);
end
end

function th = clamp_to_limits(th, limits)
th = th(:);
th = max(limits(:,1), min(limits(:,2), th));
end

function limits = urdf_joint_limits(urdfPath, jointNames)
doc = xmlread(urdfPath);
joints = doc.getElementsByTagName("joint");

n = numel(jointNames);
limits = zeros(n,2);

for i = 1:n
    name = string(jointNames(i));
    found = false;

    for j = 0:joints.getLength-1
        jt = joints.item(j);
        if string(jt.getAttribute("name")) == name
            lim = jt.getElementsByTagName("limit");
            if lim.getLength > 0
                L = lim.item(0);
                lo = str2double(string(L.getAttribute("lower")));
                hi = str2double(string(L.getAttribute("upper")));
                limits(i,:) = [lo hi];
                found = true;
            end
            break;
        end
    end

    if ~found
        % fallback: wide limits if missing
        limits(i,:) = [-pi pi];
    end
end
end

function plane = plane_fit_ls(P, pressOffset)
% Least-squares plane fit through points P (3xN)
C = mean(P,2);
X = P - C;

% normal from SVD: smallest singular vector
[~,~,V] = svd(X.', 'econ');
n = V(:,end);
n = n / norm(n);

% choose x-axis along P2-P1 projected into plane
v1 = (P(:,2) - P(:,1));
v1 = v1 - n*(n.'*v1);
if norm(v1) < 1e-9
    v1 = [0;1;0];
    v1 = v1 - n*(n.'*v1);
end
xhat = v1 / norm(v1);
yhat = cross(n, xhat);

% press offset "into the board" along -normal
p0 = C - pressOffset * n;

plane.p0   = p0;
plane.nhat = n;
plane.xhat = xhat;
plane.yhat = yhat;
plane.R    = [xhat, yhat, n];
end

function poses2 = add_pen_transitions(poses, liftExtra, N)
% Adds interpolation around pen-up moves to reduce IK jumps.
% Assumes poses is struct array with field .T
poses2 = poses;
if isempty(poses), return; end

out = poses(1);
for k = 2:numel(poses)
    Tprev = out(end).T;
    Tcur  = poses(k).T;

    pprev = Tprev(1:3,4);
    pcur  = Tcur(1:3,4);

    isJump = norm(pcur - pprev) > 0.02; % 2cm jump heuristic

    if isJump
        % lift from prev
        Tup = Tprev; Tup(3,4) = Tup(3,4) + liftExtra;
        % travel at lifted height
        Ttrav = Tcur; Ttrav(3,4) = Tup(3,4);
        % lower to new
        Tdown = Tcur;

        seg = interpolate_T(Tprev, Tup, N);
        out = [out; seg(2:end)]; %#ok<AGROW>
        seg = interpolate_T(Tup, Ttrav, N);
        out = [out; seg(2:end)]; %#ok<AGROW>
        seg = interpolate_T(Ttrav, Tdown, N);
        out = [out; seg(2:end)]; %#ok<AGROW>
    else
        out = [out; poses(k)]; %#ok<AGROW>
    end
end
poses2 = out;
end

function seg = interpolate_T(Ta, Tb, N)
seg = repmat(struct('T', eye(4)), N, 1);
pa = Ta(1:3,4); pb = Tb(1:3,4);

% keep orientation from Ta (position-only IK anyway)
R = Ta(1:3,1:3);

for i = 1:N
    s = (i-1)/(N-1);
    s = s*s*(3-2*s);
    p = (1-s)*pa + s*pb;
    T = eye(4); T(1:3,1:3) = R; T(1:3,4) = p;
    seg(i).T = T;
end
end
