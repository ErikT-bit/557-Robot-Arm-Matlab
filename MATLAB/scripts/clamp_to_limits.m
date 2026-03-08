function [theta_clamped, violated] = clamp_to_limits(theta, limits)
% clamp_to_limits
theta = theta(:);
theta_clamped = theta;
violated = false;

for i = 1:numel(theta)
    lo = limits(i,1);
    hi = limits(i,2);
    if theta_clamped(i) < lo
        theta_clamped(i) = lo; violated = true;
    elseif theta_clamped(i) > hi
        theta_clamped(i) = hi; violated = true;
    end
end
end