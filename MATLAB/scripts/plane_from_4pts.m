function plane = plane_from_4pts(P, pressOffset_m)
% plane_from_4pts
% P: 3x4 points in space frame.
% Returns plane struct with:
%   origin, x_hat, y_hat, z_hat (unit vectors)
%   origin_press (origin pushed into board by pressOffset_m)
%   coplanar_maxdist_m

if ~isequal(size(P), [3 4])
    error("P must be 3x4.");
end

p1 = P(:,1); p2 = P(:,2); p3 = P(:,3); p4 = P(:,4);

% Fit normal using two edges (more stable if you choose opposite corners)
v12 = p2 - p1;
v14 = p4 - p1;

n = cross(v12, v14);
nn = norm(n);
if nn < 1e-9
    error("Plane points degenerate: normal nearly zero.");
end
z_hat = n / nn;                 % right-hand normal

% Build x axis from v12
x_hat = v12 / norm(v12);

% y axis completes frame
y_hat = cross(z_hat, x_hat);
y_hat = y_hat / norm(y_hat);

% Re-orthogonalize x_hat (for safety)
x_hat = cross(y_hat, z_hat);
x_hat = x_hat / norm(x_hat);

origin = mean(P,2);

% Coplanarity check: distance to plane through origin
d = zeros(4,1);
for k = 1:4
    d(k) = dot(z_hat, (P(:,k) - origin));
end
maxdist = max(abs(d));
if maxdist > 0.002
    warning("Plane points not very coplanar: max dist %.3f mm", maxdist*1000);
end

% Press into board: move origin opposite the normal (into board)
origin_press = origin - pressOffset_m * z_hat;

plane = struct();
plane.origin = origin;
plane.origin_press = origin_press;

plane.x_hat = x_hat;
plane.y_hat = y_hat;
plane.z_hat = z_hat;

plane.coplanar_maxdist_m = maxdist;
end
