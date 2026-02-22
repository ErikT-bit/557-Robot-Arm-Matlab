function [goalPositions, servoIDs, logicalAngles] = moveit_rad_to_servo(theta_rad, cal)
% MOVEIT_RAD_TO_SERVO  Convert 5 MoveIt joint angles to 6 servo commands.
%
% INPUT:
%   theta_rad : 5x1 vector of joint angles in MoveIt radians
%               [base; shoulder; elbow; wrist; ee]
%   cal       : struct from servo_calibration()
%
% OUTPUT:
%   goalPositions  : 6x1 raw goal positions (0-1023 for AX-12, 0-4095 for MX)
%   servoIDs       : 6x1 servo IDs [1;2;3;4;5;6]
%   logicalAngles  : 6x1 logical angles in degrees (for debugging)
%
% USAGE:
%   cal = servo_calibration();
%   [gp, ids, la] = moveit_rad_to_servo(theta_ik, cal);

RAD2DEG = 180 / pi;
n_servos = numel(cal.servo);
logicalAngles = 180.0 * ones(n_servos, 1);   % default to home
servoIDs      = zeros(n_servos, 1);

for s = 1:n_servos
    servoIDs(s) = cal.servo(s).id;
end

% 1) Convert each MoveIt joint to its servo's logical angle
for i = 1:5
    sid = cal.moveitToServoID(i);
    logDeg = 180.0 + cal.dirSign(i) * (theta_rad(i) - cal.homeRad(i)) * RAD2DEG;

    % Find servo index for this ID
    idx = find(servoIDs == sid);
    logDeg = max(cal.servo(idx).logicalLower, min(cal.servo(idx).logicalUpper, logDeg));
    logicalAngles(idx) = logDeg;
end

% 2) Mirror: ID 3 = 360 - ID 2's logical angle
idx2 = find(servoIDs == cal.mirrorSourceID);
idx3 = find(servoIDs == cal.mirrorServoID);
logicalAngles(idx3) = 360.0 - logicalAngles(idx2);
logicalAngles(idx3) = max(cal.servo(idx3).logicalLower, ...
                      min(cal.servo(idx3).logicalUpper, logicalAngles(idx3)));

% 3) Safety tuck check for Motor 5
idx4 = find(servoIDs == 4);
idx5 = find(servoIDs == 5);
if logicalAngles(idx4) < cal.motor5TuckThreshold
    warning('Motor 4 at %.1f deg < %.1f deg threshold — tucking Motor 5 to %.1f deg.', ...
        logicalAngles(idx4), cal.motor5TuckThreshold, cal.motor5TuckAngle);
    logicalAngles(idx5) = cal.motor5TuckAngle;
end

% 4) Convert logical → physical → raw goal positions
goalPositions = zeros(n_servos, 1);
for s = 1:n_servos
    physDeg = logicalAngles(s) + (cal.servo(s).physicalOffset - 180.0);

    if cal.servo(s).isAX12
        physDeg = max(0, min(300, physDeg));
        goalPositions(s) = round(physDeg * (1023 / 300));
    else
        physDeg = max(0, min(360, physDeg));
        goalPositions(s) = round(physDeg * (4095 / 360));
    end
end
end
