function thetaTraj = run_ik_trajectory(robot, poses, theta0, limits, ev, maxIters)
% run_ik_trajectory (position-only)
% robot.Slist, robot.M
% poses: struct array with field .T
% theta0: seed (5x1)
% limits: 5x2 MoveIt limits
% ev: position tolerance (m)
% maxIters: iterations per waypoint

theta = theta0(:);
thetaTraj = zeros(numel(theta), numel(poses));

for k = 1:numel(poses)
    Tsd = poses(k).T;
    p_des = Tsd(1:3,4);

    [theta, ok] = ik_position_only_space(robot.Slist, robot.M, p_des, theta, ev, maxIters);
    if ~ok
        error("Position-only IK failed at waypoint %d / %d", k, numel(poses));
    end

    [theta, violated] = clamp_to_limits(theta, limits);
    if violated
        warning("IK waypoint %d hit joint limits; clamped.", k);
    end

    thetaTraj(:,k) = theta;
end
end
