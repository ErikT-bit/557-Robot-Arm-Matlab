function plane = plane_from_4pts(P, pressOffset)
% plane_from_4pts
% P: 3x4 points captured in 3D
% pressOffset: meters, pushes plane origin "into the board" along -z_hat

if ~isequal(size(P), [3 4])
    error("P must be 3x4.");
end

C = mean(P,2);
X = P - C;

% SVD plane fit: normal = smallest singular vector
[~,~,V] = svd(X.', 'econ');
z_hat = V(:,end);
z_hat = z_hat / norm(z_hat);

% choose x_hat from (P2 - P1) projected into plane
v1 = (P(:,2) - P(:,1));
v1 = v1 - z_hat*(z_hat.'*v1);
if norm(v1) < 1e-9
    % fallback: use (P4-P1)
    v1 = (P(:,4) - P(:,1));
    v1 = v1 - z_hat*(z_hat.'*v1);
end
x_hat = v1 / norm(v1);
y_hat = cross(z_hat, x_hat);
y_hat = y_hat / norm(y_hat);

% right-handed rotation matrix
R = [x_hat, y_hat, z_hat];

% origin pressed "into board" along -z_hat
p0 = C - pressOffset * z_hat;

% coplanarity check: max distance to plane
d = abs(z_hat.' * (P - C));
maxDist = max(d);
plane.maxPointToPlaneDist_m = maxDist;
if maxDist > 0.003
    warning("Plane points not very coplanar: max dist %.3f mm", 1000*maxDist);
end

% compute UV polygon of the four calibration points in this plane frame
UV = zeros(4,2);
for k = 1:4
    q = P(:,k) - p0;
    UV(k,1) = x_hat.' * q;
    UV(k,2) = y_hat.' * q;
end

plane.p0   = p0;
plane.x_hat = x_hat;
plane.y_hat = y_hat;
plane.z_hat = z_hat;
plane.R     = R;

plane.P = P;          % original corner points
plane.polyUV = UV;    % 4x2 UV corners relative to p0

end
