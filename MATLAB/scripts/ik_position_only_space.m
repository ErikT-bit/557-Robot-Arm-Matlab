function [theta, success] = ik_position_only_space(Slist, M, p_des, theta0, ev, maxIters)
% Position-only IK in SPACE frame:
% - Uses JacobianSpace linear part (rows 4:6)
% - Damped least squares update

theta = theta0(:);
success = false;

for it = 1:maxIters
    T = FKinSpace(M, Slist, theta);
    p = T(1:3,4);
    e = (p_des(:) - p(:));

    if norm(e) < ev
        success = true;
        return;
    end

    Js = JacobianSpace(Slist, theta); % 6xn
    Jv = Js(4:6,:);                   % 3xn

    lambda = 1e-3;
    dtheta = Jv' * ((Jv*Jv' + lambda*eye(3)) \ e);

    theta = theta + dtheta;
end
end
