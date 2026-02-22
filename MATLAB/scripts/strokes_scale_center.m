function strokes2 = strokes_scale_center(strokes, desiredHeight_m)
% strokes_scale_center
% strokes: cell array of Nx2 in normalized sketch coords
% Output is centered in XY, scaled so height becomes desiredHeight_m.

if isempty(strokes)
    error("No strokes captured.");
end

% stack points to get bounds
allPts = cell2mat(strokes(:));
xmin = min(allPts(:,1)); xmax = max(allPts(:,1));
ymin = min(allPts(:,2)); ymax = max(allPts(:,2));

w = xmax - xmin;
h = ymax - ymin;
if h < 1e-9
    error("Captured drawing has near-zero height.");
end

scale = desiredHeight_m / h;

strokes2 = cell(size(strokes));
for i = 1:numel(strokes)
    P = strokes{i};
    P(:,1) = (P(:,1) - (xmin+xmax)/2) * scale;
    P(:,2) = (P(:,2) - (ymin+ymax)/2) * scale;
    strokes2{i} = P;
end

% width check (informational)
desiredWidth_m = w * scale;
if desiredWidth_m > 0.20
    warning("Scaled drawing width = %.1f cm (may exceed your board window).", 100*desiredWidth_m);
end
end
