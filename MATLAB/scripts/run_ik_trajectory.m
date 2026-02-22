function thetaTraj = run_ik_trajectory(robot, poses, theta0, eomg, ev, maxIters)
%RUN_IK_TRAJECTORY Solve IK for a sequence of desired end-effector poses.
% Note: MR IKinSpace does not accept maxIters, so maxIters is not used here.

n = numel(theta0);

% Support poses as struct array with .T or a cell array of 4x4
if isstruct(poses)
    N = numel(poses);
    getT = @(k) poses(k).T;
elseif iscell(poses)
    N = numel(poses);
    getT = @(k) poses{k};
else
    error("poses must be a struct array (with field .T) or a cell array of 4x4 transforms");
end

thetaTraj = zeros(n, N);
theta = theta0;

for k = 1:N
    Tsd = getT(k);

    % MR IKinSpace signature: (Slist, M, T, thetalist0, eomg, ev)
    [theta, success] = IKinSpace(robot.Slist, robot.M, Tsd, theta, eomg, ev);

    if ~success
        error("IK failed at waypoint %d/%d. Try a different seed or relax tolerances.", k, N);
    end

    thetaTraj(:,k) = theta;
end
end