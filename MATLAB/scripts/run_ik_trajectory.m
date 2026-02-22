function thetaTraj = run_ik_trajectory(robot, poses, theta0, ev, maxIters)
% run_ik_trajectory
% robot:    struct with M, Slist
% poses:    struct array with poses(k).p (3x1)
% theta0:   5x1 initial guess (radians)
% ev:       position tolerance (m)
% maxIters: max iterations per waypoint
%
% FIX (22-Feb-2026):
%   1) REMOVED the local ik_position_only_space() function that was
%      SHADOWING the standalone ik_position_only_space.m.  The local
%      version used lambda=0.02, alpha=0.6, maxStep=0.10, NO line search
%      — too conservative to converge when starting from home config.
%      Now calls the standalone which has line search + adaptive damping.
%
%   2) ADDED automatic transit waypoints when the first board target is
%      far from the current FK position.  Without this, the solver tries
%      to jump from home (arm straight up, p=[0.08 0 0.59]) to the first
%      board point (p=[0.005 0.036 0.28]) — a 0.32 m gap through a
%      near-singular config.  Transit points let the solver "walk" there.

theta = theta0(:);
N = numel(poses);
n = numel(theta);

% ------------------------------------------------------------------
%  Transit waypoints: bridge the gap from current config to first target
% ------------------------------------------------------------------
T0     = FKinSpace(robot.M, robot.Slist, theta);
p_now  = T0(1:3,4);
p_first = poses(1).p(:);
gap    = norm(p_first - p_now);

TRANSIT_THRESHOLD = 0.05;   % metres — insert transit if gap > 5 cm
TRANSIT_SPACING   = 0.02;   % metres between transit points

if gap > TRANSIT_THRESHOLD
    nTransit = max(2, ceil(gap / TRANSIT_SPACING));
    fprintf("  Inserting %d transit waypoints (gap=%.3f m)\n", nTransit, gap);
    for ti = 1:nTransit
        frac  = ti / nTransit;
        p_mid = (1 - frac) * p_now + frac * p_first;

        [theta, ok] = ik_position_only_space( ...
            robot.Slist, robot.M, p_mid, theta, ev, maxIters);
        if ~ok
            T_chk = FKinSpace(robot.M, robot.Slist, theta);
            err = norm(p_mid - T_chk(1:3,4));
            warning("Transit IK stalled at point %d/%d (err=%.4f m). Continuing...", ...
                ti, nTransit, err);
        end
    end
    fprintf("  Transit complete. Starting board trajectory...\n");
end

% ------------------------------------------------------------------
%  Main trajectory solve
% ------------------------------------------------------------------
thetaTraj = zeros(n, N);

for k = 1:N
    p_des = poses(k).p(:);

    [theta, ok] = ik_position_only_space( ...
        robot.Slist, robot.M, p_des, theta, ev, maxIters);

    if ~ok
        T_chk = FKinSpace(robot.M, robot.Slist, theta);
        err = norm(p_des - T_chk(1:3,4));
        if err < ev * 5
            warning("IK marginal at waypoint %d/%d (err=%.4f m > tol=%.4f m). Using anyway.", ...
                k, N, err, ev);
        else
            fprintf("IK FAIL at waypoint %d/%d, p_des=[%.3f %.3f %.3f], err=%.4f m\n", ...
                k, N, p_des(1), p_des(2), p_des(3), err);
            error("Position-only IK failed at waypoint %d / %d (err=%.4f m)", k, N, err);
        end
    end

    thetaTraj(:,k) = theta;
end
end

% NOTE: The old version had a LOCAL function called ik_position_only_space()
% here (lines 28-57 of the original file).  That local function SHADOWED
% the standalone ik_position_only_space.m in the same directory.
% The local version was too conservative (lambda=0.02, no line search)
% to converge from home config.  It has been REMOVED so the standalone
% (which has line search + adaptive damping) gets called instead.
