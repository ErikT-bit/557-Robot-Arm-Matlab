function poses = strokes_to_taskspace(strokes, plane, penUpHeight_m)
% strokes_to_taskspace
% strokes: cell array; each cell is Nx2 stroke in meters (x,y)
% plane: from plane_from_4pts with fields origin_press, x_hat, y_hat, z_hat
% Produces poses struct array with:
%   poses(k).p    3x1 position target (m)
%   poses(k).pen  1 if pen-down, 0 if pen-up

if ~isfield(plane,"origin_press") || ~isfield(plane,"x_hat") || ~isfield(plane,"y_hat") || ~isfield(plane,"z_hat")
    error("plane struct missing required fields. Re-run plane_from_4pts rewrite.");
end

origin = plane.origin_press;
xhat = plane.x_hat; yhat = plane.y_hat; zhat = plane.z_hat;

poses = struct('p', {}, 'pen', {});
k = 0;

for s = 1:numel(strokes)
    xy = strokes{s};
    if isempty(xy), continue; end

    % Start stroke: go pen-up above first point
    p0 = origin + xhat*xy(1,1) + yhat*xy(1,2);
    k = k+1; poses(k).p = p0 + penUpHeight_m*zhat; poses(k).pen = 0;

    % Descend to pen-down
    k = k+1; poses(k).p = p0; poses(k).pen = 1;

    % Trace stroke points (pen-down)
    for i = 2:size(xy,1)
        pi = origin + xhat*xy(i,1) + yhat*xy(i,2);
        k = k+1; poses(k).p = pi; poses(k).pen = 1;
    end

    % Lift at end
    pend = origin + xhat*xy(end,1) + yhat*xy(end,2);
    k = k+1; poses(k).p = pend + penUpHeight_m*zhat; poses(k).pen = 0;
end
end
