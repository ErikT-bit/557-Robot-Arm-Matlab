function report_servo_limit_violations(raw6, cal)
% report_servo_limit_violations
% raw6: 6x1 raw counts in order cal.servoIDs = [1..6]
% cal: from servo_calibration()

raw6 = double(raw6(:));
if numel(raw6) ~= 6
    error("raw6 must be 6x1.");
end

% raw -> physical deg
physDeg = zeros(6,1);
for i = 1:6
    if cal.isAX12(i)
        physDeg(i) = raw6(i) * (300/1023);
    else
        physDeg(i) = raw6(i) * (360/4095);
    end
end

% physical -> logical
logicalDeg = physDeg - (cal.physOffset(:) - 180.0);

% check bounds
viol = false(6,1);
for i = 1:6
    if logicalDeg(i) < cal.logicalLower(i) || logicalDeg(i) > cal.logicalUpper(i)
        viol(i) = true;
    end
end

if any(viol)
    fprintf("LIMIT VIOLATIONS (logical deg):\n");
    for i = 1:6
        if viol(i)
            fprintf("  ID%d: logical=%.1f  (limit [%.1f, %.1f])  raw=%.0f\n", ...
                cal.servoIDs(i), logicalDeg(i), cal.logicalLower(i), cal.logicalUpper(i), raw6(i));
        end
    end
else
    fprintf("No motor calibration-limit violations.\n");
end

% mirror consistency check (info)
idx2 = find(cal.servoIDs == cal.mirrorSourceID, 1);
idx3 = find(cal.servoIDs == cal.mirrorTargetID, 1);
mirrorExpected = 360.0 - logicalDeg(idx2);
mirrorErr = wrap360(logicalDeg(idx3)) - wrap360(mirrorExpected);
mirrorErr = wrap180(mirrorErr);
fprintf("Mirror check: ID3 logical=%.1f, expected=%.1f, err=%.1f deg\n", ...
    logicalDeg(idx3), mirrorExpected, mirrorErr);

end

function a = wrap360(x), a = mod(x,360); end
function a = wrap180(x), a = mod(x+180,360)-180; end