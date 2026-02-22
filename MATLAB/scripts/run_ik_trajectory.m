function thetaTraj = run_ik_trajectory(robot, poses, theta0, ev, maxIters)
% run_ik_trajectory
% robot: struct with M, Slist
% poses: struct array with poses(k).p (3x1)
% theta0: 5x1 initial guess
% ev: position tolerance
% maxIters: max iterations per waypoint

theta = theta0(:);
N = numel(poses);
thetaTraj = zeros(numel(theta), N);

for k = 1:N
    p_des = poses(k).p(:);

    [theta, ok] = ik_position_only_space(robot, p_des, theta, ev, maxIters);

    if ~ok
        fprintf("IK FAIL at waypoint %d/%d, p_des=[%.3f %.3f %.3f]\n", ...
            k, N, p_des(1), p_des(2), p_des(3));
        error("Position-only IK failed at waypoint %d / %d", k, N);
    end

    thetaTraj(:,k) = theta;
end
end

function [theta, ok] = ik_position_only_space(robot, p_des, theta, ev, maxIters)
% Damped least squares on translational error only.

lambda = 0.02;     % damping
alpha  = 0.6;      % step size

ok = false;

for it = 1:maxIters
    T = FKinSpace(robot.M, robot.Slist, theta);
    p = T(1:3,4);
    e = p_des - p;

    if norm(e) < ev
        ok = true;
        return
    end

    Js = JacobianSpace(robot.Slist, theta);
    Jv = Js(4:6,:);                 % linear velocity rows

    % damped least squares step: dtheta = J'*(J*J' + l^2 I)^-1 * e
    A = (Jv*Jv' + (lambda^2)*eye(3));
    dtheta = (Jv' * (A \ e));

    % limit step to avoid jumps
    maxStep = 0.10;                 % rad
    if norm(dtheta) > maxStep
        dtheta = dtheta * (maxStep / norm(dtheta));
    end

    theta = theta + alpha*dtheta;
end
end
