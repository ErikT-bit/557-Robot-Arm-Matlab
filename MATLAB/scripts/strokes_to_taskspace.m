function poses = strokes_to_taskspace(strokes, plane, penUpHeight_m)
% strokes_to_taskspace
% strokes: cell array of Nx2 (meters in plane UV after scaling)
% plane: struct from plane_from_4pts with fields x_hat,y_hat,z_hat,p0,polyUV
% penUpHeight_m: lift distance along +z_hat (out of board)

% Tool orientation: tool Z points INTO the board.
z_tool = -plane.z_hat;
x_tool = plane.x_hat;
y_tool = cross(z_tool, x_tool);
y_tool = y_tool / norm(y_tool);

R = [x_tool, y_tool, z_tool];

% precompute polygon in UV for boundary checks
UVpoly = plane.polyUV;  % 4x2
px = UVpoly(:,1);
py = UVpoly(:,2);

poses = struct('T', {});
kout = 0;

for s = 1:numel(strokes)
    uv = strokes{s};  % Nx2, already meters in plane coords
    if size(uv,1) < 2, continue; end

    % Clamp points to polygon bounding box (simple + safe)
    umin = min(px); umax = max(px);
    vmin = min(py); vmax = max(py);

    u = uv(:,1); v = uv(:,2);
    u = max(umin, min(umax, u));
    v = max(vmin, min(vmax, v));

    % Pen-up move to first point (lifted)
    p_first = plane.p0 + plane.x_hat*u(1) + plane.y_hat*v(1) + plane.z_hat*penUpHeight_m;
    kout = kout + 1; poses(kout).T = makeT(R, p_first);

    % Lower to board (pressed plane)
    p_down = plane.p0 + plane.x_hat*u(1) + plane.y_hat*v(1);
    kout = kout + 1; poses(kout).T = makeT(R, p_down);

    % Trace stroke points on plane
    for i = 2:numel(u)
        p = plane.p0 + plane.x_hat*u(i) + plane.y_hat*v(i);
        kout = kout + 1; poses(kout).T = makeT(R, p);
    end

    % Lift at end
    p_last_up = plane.p0 + plane.x_hat*u(end) + plane.y_hat*v(end) + plane.z_hat*penUpHeight_m;
    kout = kout + 1; poses(kout).T = makeT(R, p_last_up);
end
end

function T = makeT(R,p)
T = eye(4);
T(1:3,1:3) = R;
T(1:3,4) = p(:);
end
