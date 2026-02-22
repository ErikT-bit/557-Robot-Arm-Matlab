function calibrate_limits_and_home()
% calibrate_limits_and_home
% You manually move the arm (torque OFF) to:
%   - each motor's min extreme (press ENTER)
%   - each motor's max extreme (press ENTER)
% Then you move the whole arm to HOME pose (vertical up) and press ENTER.
%
% Outputs:
%   MATLAB/calibration/servo_cal_user.mat
%   MATLAB/calibration/servo_cal_user.m  (version-controlled friendly)
%
% Notes:
% - Motors 2 & 3 are coupled (cannot move independently). We calibrate them as a pair.
% - The script records RAW ranges (counts) directly.
% - This bypasses fragile "physical/logical offset" until you have real measured home.

clc;

% ---------- SETTINGS ----------
port = "COM18";
baud = 1000000;

% Servo IDs present (your system)
ids = [1 2 3 4 5 6];

% Coupled pair (must move together)
coupledPair = [2 3];

% Servo types (true=AX 0..1023, false=MX 0..4095)
% Your earlier assumption: IDs 4,5,6 are AX; IDs 1,2,3 are MX
isAX = [false false false true true true];  % aligned with ids order above

% Direction sign in MoveIt joint order [base shoulder elbow wrist ee]
% We'll keep your existing default; you can flip later if needed.
dirSignMoveIt = [-1 +1 +1 -1 +1];

% MoveIt joints map to servo IDs (5 DOF): [base shoulder elbow wrist ee]
moveitToServoID = [1 2 4 5 6];

% ---------- CONNECT ----------
addpath(genpath(fullfile(pwd,"scripts"))); %#ok<*MCAP>
hw = robot_hw_rb150_raw6motor(port, baud);

fprintf("Torque OFF so you can move by hand...\n");
hw.torqueOff();
pause(0.25);

fprintf("\nWe will record RAW min/max for each motor.\n");
fprintf("For each prompt: move to the EXTREME, then press ENTER.\n");
fprintf("Try NOT to cross the wrap-around boundary (0) if possible.\n\n");

rawMin = nan(6,1);
rawMax = nan(6,1);

% Helper to read raw for a specific ID
    function r = readRawForID(id)
        raw6 = hw.readMotors();
        idx = find(ids==id,1);
        r = double(raw6(idx));
    end

% ---------- CALIBRATE MOTOR 1 ----------
cal_one(1);

% ---------- CALIBRATE COUPLED PAIR 2 & 3 ----------
fprintf("\n=== Coupled Pair Calibration: IDs 2 & 3 ===\n");
fprintf("IMPORTANT: Move the coupled shoulder assembly as a unit.\n");
fprintf("We will record min/max for BOTH IDs 2 and 3.\n\n");

input("Move coupled pair to MIN extreme, then press ENTER...", "s");
r2a = readRawForID(2); r3a = readRawForID(3);
fprintf("Captured MIN: ID2=%d, ID3=%d\n", round(r2a), round(r3a));

input("Move coupled pair to MAX extreme, then press ENTER...", "s");
r2b = readRawForID(2); r3b = readRawForID(3);
fprintf("Captured MAX: ID2=%d, ID3=%d\n", round(r2b), round(r3b));

[rawMin(2), rawMax(2)] = orderRange(r2a, r2b);
[rawMin(3), rawMax(3)] = orderRange(r3a, r3b);

% Mirror check informational
fprintf("Mirror check (expected roughly ID3 ~= 4095-ID2 or 1023-ID2 depending config).\n");
fprintf("This is just INFO: we will enforce your mirror constraint later.\n\n");

% ---------- CALIBRATE MOTORS 4,5,6 ----------
cal_one(4);
cal_one(5);
cal_one(6);

% ---------- CAPTURE HOME ----------
fprintf("\n========================================\n");
fprintf("Now manually move the robot to TRUE HOME pose (vertical up).\n");
fprintf("Press ENTER to record HOME raw positions for all 6 motors.\n");
fprintf("========================================\n");
input("Move to HOME pose and press ENTER...", "s");
rawHome = double(hw.readMotors());

fprintf("HOME raw:\n");
for i = 1:6
    fprintf("  ID%d: %d\n", ids(i), round(rawHome(i)));
end

% ---------- SAVE CALIBRATION ----------
cal = struct();
cal.createdAt = datestr(now);
cal.port = char(port);
cal.baud = baud;

cal.servoIDs = ids(:);
cal.isAX12   = isAX(:);

cal.rawMin  = rawMin;
cal.rawMax  = rawMax;
cal.rawHome = rawHome(:);

cal.moveitToServoID = moveitToServoID(:);
cal.dirSignMoveIt   = dirSignMoveIt(:);

cal.coupledPair = coupledPair(:);

% raw ranges sanity
bad = isnan(cal.rawMin) | isnan(cal.rawMax) | isnan(cal.rawHome);
if any(bad)
    warning("Some calibration values are NaN. Did you skip a step?");
end

% Ensure home is inside range (warn only)
for i = 1:6
    if cal.rawHome(i) < cal.rawMin(i) || cal.rawHome(i) > cal.rawMax(i)
        warning("HOME for ID%d (%d) is outside recorded range [%d,%d].", ...
            ids(i), round(cal.rawHome(i)), round(cal.rawMin(i)), round(cal.rawMax(i)));
    end
end

calDir = fullfile(pwd, "calibration");
if ~exist(calDir, "dir"), mkdir(calDir); end

matPath = fullfile(calDir, "servo_cal_user.mat");
save(matPath, "cal");

mPath = fullfile(calDir, "servo_cal_user.m");
write_cal_function(mPath, cal);

fprintf("\nSaved calibration:\n  %s\n  %s\n", matPath, mPath);

fprintf("\nYou can now TURN TORQUE ON if you want:\n");
fprintf("  hw.torqueOn();\n\n");

end

% ---------- local helpers ----------

function cal_one(id)
    fprintf("\n=== Calibrating ID%d ===\n", id);
    input(sprintf("Move ID%d to MIN extreme, then press ENTER...", id), "s");
    rA = readRawForID_local(id);
    fprintf("Captured MIN raw: ID%d=%d\n", id, round(rA));

    input(sprintf("Move ID%d to MAX extreme, then press ENTER...", id), "s");
    rB = readRawForID_local(id);
    fprintf("Captured MAX raw: ID%d=%d\n", id, round(rB));

    [mn,mx] = orderRange(rA, rB);
    assignin("caller", "rawMin", setAt(assignin_get("caller","rawMin"), id, mn));
    assignin("caller", "rawMax", setAt(assignin_get("caller","rawMax"), id, mx));
end

function x = assignin_get(ws, name)
    x = evalin(ws, name);
end

function v = setAt(v, id, val)
    ids = evalin("caller","ids");
    idx = find(ids==id,1);
    v(idx) = val;
end

function r = readRawForID_local(id)
    hw = evalin("caller","hw");
    ids = evalin("caller","ids");
    raw6 = hw.readMotors();
    idx = find(ids==id,1);
    r = double(raw6(idx));
end

function [mn,mx] = orderRange(a,b)
    mn = min(a,b); mx = max(a,b);
end

function write_cal_function(path, cal)
    fid = fopen(path, "w");
    fprintf(fid, "function cal = servo_cal_user()\n");
    fprintf(fid, "%% Auto-generated by calibrate_limits_and_home.m on %s\n\n", cal.createdAt);

    fprintf(fid, "cal.createdAt = '%s';\n", cal.createdAt);
    fprintf(fid, "cal.port = '%s';\n", cal.port);
    fprintf(fid, "cal.baud = %d;\n", cal.baud);

    fprintf(fid, "cal.servoIDs = [%s]';\n", numlist(cal.servoIDs));
    fprintf(fid, "cal.isAX12   = [%s]';\n", numlist(double(cal.isAX12)));

    fprintf(fid, "cal.rawMin  = [%s]';\n", numlist(cal.rawMin));
    fprintf(fid, "cal.rawMax  = [%s]';\n", numlist(cal.rawMax));
    fprintf(fid, "cal.rawHome = [%s]';\n", numlist(cal.rawHome));

    fprintf(fid, "cal.moveitToServoID = [%s]';\n", numlist(cal.moveitToServoID));
    fprintf(fid, "cal.dirSignMoveIt   = [%s]';\n", numlist(cal.dirSignMoveIt));

    fprintf(fid, "cal.coupledPair = [%s]';\n", numlist(cal.coupledPair));
    fprintf(fid, "end\n");
    fclose(fid);
end

function s = numlist(v)
    v = double(v(:))';
    s = sprintf("%.0f ", v);
    s = strtrim(s);
end