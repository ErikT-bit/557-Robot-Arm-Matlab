function poses = strokes_to_taskspace(strokes, plane, penUpHeight_m, boardBackoff_m)
% strokes_to_taskspace
% strokes: cell array; each cell is Nx2 stroke in meters (x,y)
% plane: from plane_from_4pts with fields origin_press, x_hat, y_hat, z_hat
%
% Produces poses struct array with:
%   poses(k).p    3x1 position target (m)
%   poses(k).pen  1 if pen-down, 0 if pen-up
%
% Current conventions:
%   - sketch Y is flipped so letters are upright
%   - pen-up pulls back toward the robot along -z_hat
%   - origin_write includes both an intentional board offset and a
%     measured contact-bias correction

if nargin < 4
    boardBackoff_m = 0;
end

if ~isfield(plane,"origin_press") || ~isfield(plane,"x_hat") || ...
   ~isfield(plane,"y_hat") || ~isfield(plane,"z_hat")
    error("plane struct missing required fields.");
end

origin = plane.origin_press;
xhat = plane.x_hat(:);
yhat = plane.y_hat(:);
zhat = plane.z_hat(:) / norm(plane.z_hat);

% ---- ORIENTATION FIX ----
flipSketchY = true;

% ---- CONTACT CALIBRATION ----
% If the robot writes consistently 3 mm off the board, correct it here.
% Positive value here means "push the writing path 3 mm farther toward
% the board than the current model would otherwise place it."
contactBias_m = 0.003;

% ---- WRITE PLANE OFFSET ----
% Keep your currently working board-backoff sign and apply the measured bias.
origin_write = origin - boardBackoff_m * zhat + contactBias_m * zhat;

poses = struct('p', {}, 'pen', {});
k = 0;

for s = 1:numel(strokes)
    xy = strokes{s};
    if isempty(xy), continue; end

    if flipSketchY
        xy_use = xy;
        xy_use(:,2) = -xy_use(:,2);
    else
        xy_use = xy;
    end

    % Start stroke point on writing plane
    p0 = origin_write + xhat*xy_use(1,1) + yhat*xy_use(1,2);

    % Move pen-up to start point, pulled back toward robot
    k = k + 1;
    poses(k).p = p0 - penUpHeight_m * zhat;
    poses(k).pen = 0;

    % Descend to writing surface
    k = k + 1;
    poses(k).p = p0;
    poses(k).pen = 1;

    % Trace stroke points on writing plane
    for i = 2:size(xy_use,1)
        pi = origin_write + xhat*xy_use(i,1) + yhat*xy_use(i,2);
        k = k + 1;
        poses(k).p = pi;
        poses(k).pen = 1;
    end

    % Lift at end, back toward robot
    pend = origin_write + xhat*xy_use(end,1) + yhat*xy_use(end,2);
    k = k + 1;
    poses(k).p = pend - penUpHeight_m * zhat;
    poses(k).pen = 0;
end
end