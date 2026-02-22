function raw6 = moveit_rad_to_servo(theta5, cal)
% moveit_rad_to_servo
% theta5: 5x1 MoveIt joints [base shoulder elbow wrist pen] in radians
% raw6: 6x1 raw goals ordered by cal.servoIDs

theta5 = double(theta5(:));
if numel(theta5) ~= 5
    error("theta5 must be 5x1.");
end

ids = cal.servoIDs(:);
raw6 = double(cal.rawHome(:));   % start at home raw

% Convert each MoveIt joint to its servo raw (using home + delta counts)
for j = 1:5
    sid = cal.moveitToServoID(j);
    idx = find(ids == sid, 1);

    Rmax = cal.rawRangeMax(idx);

    % counts per rad
    if cal.isAX12(idx)
        counts_per_rad = (1023/300) * (180/pi);   % AX: 300° span over 1023
    else
        counts_per_rad = (4095/360) * (180/pi);   % MX: 360° span over 4095
    end

    dCounts = cal.dirSignMoveIt(j) * theta5(j) * counts_per_rad;
    rawCmd  = cal.rawHome(idx) + dCounts;

    % wrap + clamp
    rawCmd = wrap_raw(rawCmd, Rmax);
    rawCmd = clamp_raw_allowed(rawCmd, cal.allowedMin(idx), cal.allowedMax(idx), Rmax);

    raw6(idx) = rawCmd;
end

% ---- Coupled pair enforcement: ID3 leads, ID2 mirrors ----
idx2 = find(ids == cal.coupledMirrorID, 1); % ID2
idx3 = find(ids == cal.coupledLeadID,   1); % ID3

Rmax3 = cal.rawRangeMax(idx3);
Rmax2 = cal.rawRangeMax(idx2);

% delta of leader from its home (shortest signed delta in counts)
d3 = shortest_delta_counts(raw6(idx3), cal.rawHome(idx3), Rmax3);

% mirror means opposite delta around its own home
raw2 = cal.rawHome(idx2) - d3;

raw2 = wrap_raw(raw2, Rmax2);
raw2 = clamp_raw_allowed(raw2, cal.allowedMin(idx2), cal.allowedMax(idx2), Rmax2);

raw6(idx2) = raw2;

end

% ---------- helpers ----------
function r = wrap_raw(r, Rmax)
r = mod(round(r), Rmax+1);
end

function d = shortest_delta_counts(r, r0, Rmax)
% returns signed delta in counts from r0 to r in [-R/2, R/2]
r  = wrap_raw(r,  Rmax);
r0 = wrap_raw(r0, Rmax);
d = r - r0;
half = (Rmax+1)/2;
if d > half,  d = d - (Rmax+1); end
if d < -half, d = d + (Rmax+1); end
end

function r = clamp_raw_allowed(r, mn, mx, Rmax)
% clamps r to nearest boundary if outside allowed (wrap-aware)
r  = wrap_raw(r,  Rmax);
mn = wrap_raw(mn, Rmax);
mx = wrap_raw(mx, Rmax);

if mn <= mx
    if r < mn, r = mn; end
    if r > mx, r = mx; end
else
    % allowed: [mn..Rmax] U [0..mx]
    if ~(r >= mn || r <= mx)
        % outside: decide nearest boundary by circular distance
        d_to_mn = circ_dist(r, mn, Rmax);
        d_to_mx = circ_dist(r, mx, Rmax);
        if d_to_mn < d_to_mx
            r = mn;
        else
            r = mx;
        end
    end
end
end

function d = circ_dist(a,b,Rmax)
% minimal circular distance in counts
a = wrap_raw(a,Rmax); b = wrap_raw(b,Rmax);
d = abs(a-b);
d = min(d, (Rmax+1)-d);
end
