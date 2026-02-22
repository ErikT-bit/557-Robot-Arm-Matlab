function thetaTraj = run_ik_trajectory(robot, poses, theta0, limits, ev, maxIters)
% run_ik_trajectory (position-only, robust)
theta = theta0(:);
n = numel(theta);
thetaTraj = zeros(n, numel(poses));

for k = 1:numel(poses)
    p_des = poses(k).T(1:3,4);

    [theta_new, ok] = ik_position_only_space(robot.Slist, robot.M, p_des, theta, ev, maxIters);

    % If first waypoint fails, try again from home seed
    if ~ok && k == 1
        [theta_new, ok] = ik_position_only_space(robot.Slist, robot.M, p_des, theta0(:), ev, maxIters*2);
    end

    if ~ok
        fprintf("IK FAIL at waypoint %d/%d, p_des=[%.3f %.3f %.3f]\n", ...
            k, numel(poses), p_des(1), p_des(2), p_des(3));
        error("Position-only IK failed at waypoint %d / %d", k, numel(poses));
    end

    [theta_new, violated] = clamp_to_limits(theta_new, limits);
    if violated
        warning("IK waypoint %d hit MoveIt joint limits; clamped.", k);
    end

    theta = theta_new;
    thetaTraj(:,k) = theta;
end
end
