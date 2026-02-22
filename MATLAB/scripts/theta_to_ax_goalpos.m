function goalPos = theta_to_ax_goalpos(theta_rad, mid, dirSign)
% Convert joint angle (rad) to AX-style goal position (0..1023).
% mid = 512 usually, dirSign = +1 or -1 depending on motor mounting.

countsPerRad = 1023 / (300 * pi/180);  % 300 deg range
goalPos = mid + dirSign * theta_rad * countsPerRad;
goalPos = max(0, min(1023, round(goalPos)));
end