function plane = plane_from_4pts(P, pressOffset)
% P is 3x4 points on the *actual* board surface captured by robot FK.
% We define a plane frame:
%   origin = centroid of points
%   x_hat = along P1->P2
%   z_hat = plane normal (right-hand)
%   y_hat = z_hat x x_hat
% Then shift origin by -pressOffset*z_hat so pen presses slightly into surface.

p1 = P(:,1); p2 = P(:,2); p4 = P(:,4);

origin = mean(P,2);

x_hat = Normalize(p2 - p1);

% normal from two non-collinear directions
n = cross(p2 - p1, p4 - p1);
z_hat = Normalize(n);

% complete right-hand frame
y_hat = Normalize(cross(z_hat, x_hat));
x_hat = Normalize(cross(y_hat, z_hat)); % re-orthonormalize

origin_pressed = origin - pressOffset * z_hat;

plane = struct();
plane.origin_surface = origin;
plane.origin_pressed = origin_pressed;
plane.x_hat = x_hat;
plane.y_hat = y_hat;
plane.z_hat = z_hat;
plane.pressOffset = pressOffset;
end
