function [theta, success] = ik_position_only_space(Slist, M, p_des, theta0, ev, maxIters)
% ik_position_only_space  Position-only IK using damped least squares.
%
% Solves for joint angles that place the end-effector at p_des, ignoring
% orientation.  Uses damped least squares on the 3xn linear-velocity
% Jacobian with line search and adaptive damping for robustness.
%
% INPUT:
%   Slist    : 6xn space-frame screw axes
%   M        : 4x4 home configuration
%   p_des    : 3x1 desired position (metres)
%   theta0   : nx1 initial guess
%   ev       : position error tolerance (m), e.g. 1e-3
%   maxIters : max Newton iterations (e.g. 200)
%
% OUTPUT:
%   theta   : nx1 solution
%   success : true if norm(p_des - FK(theta)) < ev
%
% FIX (22-Feb-2026):
%   Previous version had lambda=1e-3, maxStep=0.15, basic line search.
%   This version adds:
%     - Adaptive damping (increases on failure, decreases on success)
%     - Wider line search factors
%     - Retry with perturbed seeds if primary solve stalls
%   This matters when starting from near-singular configs (e.g. home =
%   arm straight up) and targeting points far away on the board.

theta   = theta0(:);
p_des   = p_des(:);
success = false;

% ---- tuning ----
lambda0    = 5e-3;
lambda_min = 1e-4;
lambda_max = 0.5;
maxStep    = 0.20;         % rad per iteration cap
alphaList  = [1.0 0.5 0.25 0.1 0.05];

lambda = lambda0;

% ---------- primary solve ----------
[theta, success] = solve_inner(theta, lambda);
if success, return; end

% ---------- retry with perturbed seeds ----------
rng_state = rng;
rng(42);
for attempt = 1:4
    perturbation = 0.3 * randn(numel(theta0), 1);
    theta_try = theta0(:) + perturbation;
    [theta_try, ok] = solve_inner(theta_try, lambda0);
    if ok
        theta = theta_try;
        success = true;
        rng(rng_state);
        return;
    end
end
rng(rng_state);

% Return best result even if not converged — caller checks `success`

    function [th, ok] = solve_inner(th, lam)
        ok = false;
        for it = 1:maxIters
            T = FKinSpace(M, Slist, th);
            p = T(1:3,4);
            e = p_des - p;
            enorm = norm(e);

            if enorm < ev
                ok = true;
                return;
            end

            Js = JacobianSpace(Slist, th);   % 6xn
            Jv = Js(4:6,:);                  % 3xn (linear velocity rows)

            % Damped least squares step
            A  = Jv * Jv' + (lam^2) * eye(3);
            dtheta = Jv' * (A \ e);

            % Cap step size
            nrm = norm(dtheta);
            if nrm > maxStep
                dtheta = dtheta * (maxStep / nrm);
            end

            % Backtracking line search
            accepted = false;
            for a = alphaList
                th_try = th + a * dtheta;
                T_try  = FKinSpace(M, Slist, th_try);
                e_try  = norm(p_des - T_try(1:3,4));
                if e_try < enorm
                    th = th_try;
                    accepted = true;
                    lam = max(lambda_min, lam * 0.8);   % decrease damping
                    break;
                end
            end

            if ~accepted
                lam = min(lambda_max, lam * 3.0);       % increase damping
                th  = th + 0.05 * dtheta;               % tiny nudge
            end
        end
    end
end
