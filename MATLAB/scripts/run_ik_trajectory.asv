function thetaTraj = run_ik_trajectory(robot, poses, theta_seed, ev, maxIters)
% run_ik_trajectory
% Position-only damped least-squares IK for a sequence of task-space points.
%
% Inputs:
%   robot      struct with fields M, Slist
%   poses      1xN struct array, each with at least poses(i).p = [x;y;z]
%   theta_seed nx1 initial guess
%   ev         position tolerance in meters
%   maxIters   max iterations per waypoint
%
% Output:
%   thetaTraj  nxN solved joint trajectory
%
% Notes:
%   Uses the SPACE Jacobian, but correctly converts it to the Jacobian of
%   the end-effector origin position:
%
%       p_dot = omega x p + v
%
%   so
%
%       Jp = -skew(p)*Jw + Jv
%
%   where Jw = Js(1:3,:), Jv = Js(4:6,:)

if isempty(poses)
    thetaTraj = zeros(numel(theta_seed),0);
    return
end

theta_seed = theta_seed(:);
n = numel(theta_seed);
Nw = numel(poses);

thetaTraj = zeros(n, Nw);

% ---- solver tuning ----
lambda = 0.02;                % DLS damping
stepMax = deg2rad(4);         % max per-joint step per iteration
maxWaypointJump = 0.020;      % split large Cartesian gaps into smaller subtargets
stallTol = 1e-6;              % tiny improvement threshold
stallCountMax = 15;           % consecutive tiny-improvement iterations before stopping

theta_curr = theta_seed;

for i = 1:Nw
    p_des = poses(i).p(:);

    % current pose
    T_curr = FKinSpace(robot.M, robot.Slist, theta_curr);
    p_curr = T_curr(1:3,4);

    gap = norm(p_des - p_curr);

    if gap > maxWaypointJump
        nSub = ceil(gap / maxWaypointJump);
        subPts = zeros(3, nSub);
        for s = 1:nSub
            alpha = s / nSub;
            subPts(:,s) = (1-alpha)*p_curr + alpha*p_des;
        end
    else
        nSub = 1;
        subPts = p_des;
    end

    for s = 1:nSub
        p_sub = subPts(:,s);

        [theta_sol, err_norm, success] = solve_one_position_target( ...
            robot, theta_curr, p_sub, ev, maxIters, lambda, stepMax, stallTol, stallCountMax);

        if ~success
            fprintf('IK FAIL at waypoint %d/%d, p_des=[%.3f %.3f %.3f], err=%.4f m\n', ...
                i, Nw, p_sub(1), p_sub(2), p_sub(3), err_norm);
            error('Position-only IK failed at waypoint %d / %d (err=%.4f m)', i, Nw, err_norm);
        end

        theta_curr = theta_sol;
    end

    thetaTraj(:,i) = theta_curr;
end
end

% #####################################################################
% LOCAL SOLVER
% #####################################################################

function [theta, err_norm, success] = solve_one_position_target( ...
    robot, theta0, p_des, ev, maxIters, lambda, stepMax, stallTol, stallCountMax)

theta = theta0(:);
prevErr = inf;
stallCount = 0;
success = false;

for k = 1:maxIters
    Tsb = FKinSpace(robot.M, robot.Slist, theta);
    p_now = Tsb(1:3,4);

    e = p_des - p_now;
    err_norm = norm(e);

    if err_norm <= ev
        success = true;
        return
    end

    Js = JacobianSpace(robot.Slist, theta);

    Jw = Js(1:3,:);
    Jv = Js(4:6,:);

    % Correct translational Jacobian for end-effector origin in space frame:
    % p_dot = omega x p + v = -skew(p)*omega + v
    Jp = -VecToso3(p_now) * Jw + Jv;

    % Damped least-squares
    A = Jp * Jp' + (lambda^2) * eye(3);
    dtheta = Jp' * (A \ e);

    % clamp step size
    dtheta = max(min(dtheta, stepMax), -stepMax);

    theta = theta + dtheta;

    % stall detection
    if abs(prevErr - err_norm) < stallTol
        stallCount = stallCount + 1;
    else
        stallCount = 0;
    end
    prevErr = err_norm;

    if stallCount >= stallCountMax
        break
    end
end

Tsb = FKinSpace(robot.M, robot.Slist, theta);
p_now = Tsb(1:3,4);
err_norm = norm(p_des - p_now);
success = err_norm <= ev;
end