function [theta, success] = ik_position_only_space(Slist, M, p_des, theta0, ev, maxIters)
% More robust position-only IK (SPACE frame)
% - damped least squares
% - step limiting
% - simple line-search on position error

theta = theta0(:);
success = false;

lambda = 1e-3;
maxStep = 0.15;      % rad per iteration cap (prevents jumps)
alphaList = [1.0 0.5 0.25 0.1];  % line search factors

for it = 1:maxIters
    T = FKinSpace(M, Slist, theta);
    p = T(1:3,4);
    e = p_des(:) - p(:);

    if norm(e) < ev
        success = true;
        return;
    end

    Js = JacobianSpace(Slist, theta); % 6xn
    Jv = Js(4:6,:);                   % 3xn

    % dls step
    dtheta = Jv' * ((Jv*Jv' + lambda*eye(3)) \ e);

    % cap step size
    nrm = norm(dtheta);
    if nrm > maxStep
        dtheta = dtheta * (maxStep / nrm);
    end

    % line search: accept first that reduces error
    accepted = false;
    for a = alphaList
        th_try = theta + a*dtheta;
        Ttry = FKinSpace(M, Slist, th_try);
        etry = p_des(:) - Ttry(1:3,4);
        if norm(etry) < norm(e)
            theta = th_try;
            accepted = true;
            break;
        end
    end

    if ~accepted
        % if nothing improves, increase damping slightly and still take tiny step
        lambda = min(1e-1, lambda*5);
        theta = theta + 0.1*dtheta;
    end
end
end
