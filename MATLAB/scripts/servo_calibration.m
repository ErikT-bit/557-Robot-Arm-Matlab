function cal = servo_calibration()
% servo_calibration
% Loads user calibration if available:
%   MATLAB/calibration/servo_cal_user.mat
% Otherwise errors (we require calibration now).

thisFile = mfilename('fullpath');
scriptsDir = fileparts(thisFile);
matlabDir  = fileparts(scriptsDir);

calPath = fullfile(matlabDir, "calibration", "servo_cal_user.mat");
if ~exist(calPath, "file")
    error("Calibration not found: %s. Run calibrate_limits_and_home first.", calPath);
end

S = load(calPath, "cal");
cal = S.cal;

% ---- Derive wrap-aware limit representation ----
cal = derive_wrap_limits(cal);

% ---- Define coupled behavior: motor 3 leads, motor 2 mirrors ----
% Your statement: "motor 3 gets the input, motor 2 copies but mirrors"
cal.coupledLeadID   = 3;    % leader
cal.coupledMirrorID = 2;    % mirrored follower

% MoveIt joint mapping (5 DOF): [base shoulder elbow wrist pen]
% We will now map SHOULDER to the LEADER (ID3), not ID2.
cal.moveitToServoID = [1 3 4 5 6]';  % (base, shoulder, elbow, wrist, pen)

end

function cal = derive_wrap_limits(cal)
% Creates per-ID wrap-aware "allowed interval" around HOME.
% If rawMin <= rawMax => normal interval [min,max]
% If rawMin > rawMax  => wrap interval: [min..MAX] U [0..max]

ids  = cal.servoIDs(:);
isAX = logical(cal.isAX12(:));

rawMin  = double(cal.rawMin(:));
rawMax  = double(cal.rawMax(:));
rawHome = double(cal.rawHome(:));

cal.rawRangeMax = zeros(numel(ids),1); % 1023 or 4095
for i = 1:numel(ids)
    cal.rawRangeMax(i) = isAX(i) * 1023 + (~isAX(i)) * 4095;
end

cal.wraps = rawMin > rawMax;
cal.allowedMin = rawMin;
cal.allowedMax = rawMax;

% sanity: ensure home is inside the allowed interval (wrap-aware)
for i = 1:numel(ids)
    if ~raw_in_allowed(rawHome(i), rawMin(i), rawMax(i), cal.rawRangeMax(i))
        warning("HOME raw for ID%d (%d) is outside your allowed interval (wrap-aware).", ...
            ids(i), round(rawHome(i)));
    end
end
end

function ok = raw_in_allowed(r, mn, mx, Rmax)
r  = wrap_raw(r, Rmax);
mn = wrap_raw(mn, Rmax);
mx = wrap_raw(mx, Rmax);
if mn <= mx
    ok = (r >= mn) && (r <= mx);
else
    ok = (r >= mn) || (r <= mx); % wrap interval
end
end

function r = wrap_raw(r, Rmax)
r = mod(round(r), Rmax+1);
end
