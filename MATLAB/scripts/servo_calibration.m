function cal = servo_calibration()
% servo_calibration
% Loads calibration from MATLAB/calibration/servo_cal_user.mat (REQUIRED),
% derives wrap-aware limits, and defines coupled motor metadata expected by
% moveit_rad_to_servo / servo_to_moveit_rad.

thisFile   = mfilename('fullpath');
scriptsDir = fileparts(thisFile);
matlabDir  = fileparts(scriptsDir);

calPath = fullfile(matlabDir, "calibration", "servo_cal_user.mat");
if ~exist(calPath, "file")
    error("Calibration not found: %s. Run your calibration script first.", calPath);
end

S = load(calPath, "cal");
if ~isfield(S, "cal")
    error("File exists but does not contain variable 'cal': %s", calPath);
end
cal = S.cal;

% ---- Validate required fields ----
req = ["servoIDs","isAX12","rawMin","rawMax"];
for k = 1:numel(req)
    if ~isfield(cal, req(k))
        error("Calibration missing required field cal.%s", req(k));
    end
end

cal.servoIDs = double(cal.servoIDs(:));
cal.isAX12   = logical(cal.isAX12(:));
cal.rawMin   = double(cal.rawMin(:));
cal.rawMax   = double(cal.rawMax(:));

% rawHome may be stale after motor swaps; keep it but do not enforce limits
if ~isfield(cal,"rawHome") || numel(cal.rawHome) ~= 6
    cal.rawHome = nan(6,1);
else
    cal.rawHome = double(cal.rawHome(:));
end

% Defaults if missing
if ~isfield(cal,"moveitToServoID") || numel(cal.moveitToServoID) ~= 5
    cal.moveitToServoID = [1; 2; 4; 5; 6];
else
    cal.moveitToServoID = double(cal.moveitToServoID(:));
end

if ~isfield(cal,"dirSignMoveIt") || numel(cal.dirSignMoveIt) ~= 5
    cal.dirSignMoveIt = [-1; +1; +1; -1; +1];
else
    cal.dirSignMoveIt = double(cal.dirSignMoveIt(:));
end

% ---- Derive wrap-aware limit representation (from rawMin/rawMax only) ----
cal = derive_wrap_limits(cal);

% ---- Coupled metadata EXPECTED by your conversion functions ----
% Your earlier design: motor 3 is the "leader", motor 2 is the mirrored follower.
% These field names are what moveit_rad_to_servo.m is looking for.
cal.coupledLeadID   = 3;
cal.coupledMirrorID = 2;

end

function cal = derive_wrap_limits(cal)
ids  = cal.servoIDs(:);
isAX = logical(cal.isAX12(:));

rawMin = double(cal.rawMin(:));
rawMax = double(cal.rawMax(:));

cal.rawRangeMax = zeros(numel(ids),1); % 1023 or 4095
for i = 1:numel(ids)
    if isAX(i)
        cal.rawRangeMax(i) = 1023;
    else
        cal.rawRangeMax(i) = 4095;
    end
end

cal.wraps      = rawMin > rawMax;
cal.allowedMin = rawMin;
cal.allowedMax = rawMax;
end