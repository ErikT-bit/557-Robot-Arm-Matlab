function theta_rad = servo_to_moveit_rad(rawPos, cal)
% servo_to_moveit_rad
% Convert 6 raw servo present positions -> 5 MoveIt joint radians.
% rawPos: 6x1 in order IDs [1 2 3 4 5 6] (same order as cal.servoIDs)

rawPos = double(rawPos(:));
if numel(rawPos) ~= 6
    error("rawPos must be 6x1 corresponding to IDs [1..6].");
end

servoIDs = cal.servoIDs(:);

% 1) raw -> physical degrees
physDeg = zeros(6,1);
for i = 1:6
    if cal.isAX12(i)
        physDeg(i) = rawPos(i) * (300/1023);
    else
        physDeg(i) = rawPos(i) * (360/4095);
    end
end

% 2) physical -> logical
% phys = logical + (offset - 180)  => logical = phys - (offset - 180)
logicalAngles = physDeg - (cal.physOffset(:) - 180.0);

% 3) mirror check (ID3 should be 360 - ID2)
idx2 = find(servoIDs == cal.mirrorSourceID, 1);
idx3 = find(servoIDs == cal.mirrorTargetID, 1);
mirrorExpected = 360.0 - logicalAngles(idx2);
if abs(wrap360(logicalAngles(idx3)) - wrap360(mirrorExpected)) > 6.0
    warning("Mirror mismatch: ID3 logical=%.1f deg, expected %.1f deg (from ID2).", ...
        logicalAngles(idx3), mirrorExpected);
end

% 4) logical -> MoveIt radians (only for the 5 moveit joints)
theta_rad = zeros(5,1);
RAD = pi/180;

for j = 1:5
    sid = cal.moveitToServoID(j);
    idx = find(servoIDs == sid, 1);

    % logDeg = 180 + dirSign * theta * RAD2DEG
    % => theta = dirSign * (logDeg - 180) * DEG2RAD
    theta_rad(j) = cal.dirSign(j) * (logicalAngles(idx) - 180.0) * RAD;
end
end

function a = wrap360(x)
a = mod(x, 360);
end
