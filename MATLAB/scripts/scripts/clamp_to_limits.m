function [theta_clamped, violated] = clamp_to_limits(theta, limits)
% CLAMP_TO_LIMITS  Clamp joint angles to physical limits.
%
% INPUT:
%   theta  : Nx1 vector of joint angles (radians)
%   limits : struct from robot_joint_limits() with fields .lower, .upper
%
% OUTPUT:
%   theta_clamped : Nx1 clamped joint angles
%   violated      : logical — true if any joint was out of bounds
%
% USAGE:
%   limits = robot_joint_limits();
%   [theta, bad] = clamp_to_limits(theta_ik, limits);
%   if bad, warning('IK solution hit joint limits'); end

theta_clamped = max(limits.lower, min(limits.upper, theta));
violated = any(abs(theta_clamped - theta) > 1e-10);

if violated
    diffs = theta - theta_clamped;
    for i = 1:numel(theta)
        if abs(diffs(i)) > 1e-10
            fprintf('  Joint %d (%s): IK=%.4f rad, clamped to [%.4f, %.4f]\n', ...
                i, limits.names(i), theta(i), limits.lower(i), limits.upper(i));
        end
    end
end
end
