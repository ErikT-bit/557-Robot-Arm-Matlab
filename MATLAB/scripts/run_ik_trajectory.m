function thetaTraj = run_ik_trajectory(robot, poses, theta0, eomg, ev, maxIters)
% Sequential IK solve (warm start):
% theta_{i+1} starts from theta_i so it converges quickly.

n = size(robot.Slist,2);
thetaTraj = zeros(n, numel(poses));
theta = theta0;

for i = 1:numel(poses)
    Tsd = poses{i};

    [theta, success] = IKinSpace(robot.Slist, robot.M, Tsd, theta, eomg, ev, maxIters);

    if ~success
        error("IK failed at pose %d. Try smaller drawing / better plane / looser tolerances.", i);
    end

    thetaTraj(:,i) = theta;
end
end