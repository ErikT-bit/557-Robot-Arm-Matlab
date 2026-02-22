function theta5 = servo_to_moveit_rad(raw6, cal)
% servo_to_moveit_rad
% raw6: 6x1 raw positions ordered by cal.servoIDs
% theta5: 5x1 MoveIt joints [base shoulder elbow wrist pen] in radians

raw6 = double(raw6(:));
if numel(raw6) ~= 6
    error("raw6 must be 6x1.");
end

ids = cal.servoIDs(:);
theta5 = zeros(5,1);

for j = 1:5
    sid = cal.moveitToServoID(j);
    idx = find(ids == sid, 1);

    Rmax = cal.rawRangeMax(idx);

    if cal.isAX12(idx)
        counts_per_rad = (1023/300) * (180/pi);
    else
        counts_per_rad = (4095/360) * (180/pi);
    end

    d = shortest_delta_counts(raw6(idx), cal.rawHome(idx), Rmax); % signed counts
    theta5(j) = (d / counts_per_rad) * cal.dirSignMoveIt(j);
end

end

function r = wrap_raw(r, Rmax)
r = mod(round(r), Rmax+1);
end

function d = shortest_delta_counts(r, r0, Rmax)
r  = wrap_raw(r,  Rmax);
r0 = wrap_raw(r0, Rmax);
d = r - r0;
half = (Rmax+1)/2;
if d > half,  d = d - (Rmax+1); end
if d < -half, d = d + (Rmax+1); end
end
