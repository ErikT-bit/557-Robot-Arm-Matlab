function [theta, success] = ik_position_only_space(Slist, M, p_des, theta0, ev, maxIters)
% Position-only IK in SPACE frame using JacobianSpace linear part.
% Slist: 6xn
% M: 4x4
% p_des: 3x1 desired position
% theta0: nx1 seed
% ev: position tolerance (m)
% maxIters: iterations

theta = theta0(:);
n = numel(theta);
success = false;

for i = 1:maxIters
    T = FKinSpace(M, Slist, theta);
    p = T(1:3,4);
    e = (p_des(:) - p(:));

    if norm(e) < ev
        success = true;
        return;
    end

    Js = JacobianSpace(Slist, theta);     % 6xn
    Jv = Js(4:6,:);                       % linear part (3xn)

    % damped least squares for robustness
    lambda = 1e-3;
    dtheta = (Jv' * ((Jv*Jv' + lambda*eye(3)) \ e));

    if numel(dtheta) ~= n
        dtheta = pinv(Jv) * e;
    end

    theta = theta + dtheta;
end
end