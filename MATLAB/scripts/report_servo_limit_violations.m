function report_servo_limit_violations(raw6, cal)
raw6 = double(raw6(:));
if numel(raw6) ~= 6, error("raw6 must be 6x1."); end

ids = cal.servoIDs(:);
viol = false(6,1);

for i = 1:6
    Rmax = cal.rawRangeMax(i);
    r  = wrap_raw(raw6(i), Rmax);
    mn = wrap_raw(cal.allowedMin(i), Rmax);
    mx = wrap_raw(cal.allowedMax(i), Rmax);

    if mn <= mx
        ok = (r >= mn) && (r <= mx);
    else
        ok = (r >= mn) || (r <= mx);
    end
    viol(i) = ~ok;
end

if any(viol)
    fprintf("LIMIT VIOLATIONS (raw counts):\n");
    for i = 1:6
        if viol(i)
            fprintf("  ID%d: raw=%d  allowed=[%d..%d]%s\n", ...
                ids(i), round(raw6(i)), round(cal.allowedMin(i)), round(cal.allowedMax(i)), ...
                tern(cal.wraps(i)," (wrap)",""));
        end
    end
else
    fprintf("No motor RAW-limit violations.\n");
end

% Coupled check info: is ID2 mirroring ID3 around their homes?
idx2 = find(ids==cal.coupledMirrorID,1);
idx3 = find(ids==cal.coupledLeadID,1);
d3 = shortest_delta_counts(raw6(idx3), cal.rawHome(idx3), cal.rawRangeMax(idx3));
d2 = shortest_delta_counts(raw6(idx2), cal.rawHome(idx2), cal.rawRangeMax(idx2));
fprintf("Coupled check: d3=%+.0f counts, d2=%+.0f counts (expect d2≈-d3)\n", d3, d2);

end

function s = tern(cond,a,b)
if cond, s=a; else, s=b; end
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
