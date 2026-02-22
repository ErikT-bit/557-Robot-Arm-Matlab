function strokesOut = strokes_scale_center(strokes, desiredHeight_m)
% Scales strokes so total height = desiredHeight_m and centers at (0,0).
%
% Input strokes{k}.xy is 2xN in arbitrary sketch units.
% Output strokesOut{k}.xy is 2xN in meters (scaled).

allPts = [];
for k = 1:numel(strokes)
    allPts = [allPts, strokes{k}.xy]; %#ok<AGROW>
end

minXY = min(allPts,[],2);
maxXY = max(allPts,[],2);
height = maxXY(2) - minXY(2);

if height < 1e-9
    error("Drawing height is ~zero. Draw something with vertical extent.");
end

scale = desiredHeight_m / height;
center = (minXY + maxXY)/2;

strokesOut = strokes;
for k = 1:numel(strokes)
    xy = strokes{k}.xy;
    xy = (xy - center) * scale;
    strokesOut{k}.xy = xy;
end
end