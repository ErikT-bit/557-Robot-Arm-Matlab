function raw6 = moveit_rad_to_servo(theta5, cal)
% moveit_rad_to_servo
% Converts 5 MoveIt joint angles (rad) -> 6 raw servo goals.
%
% MoveIt joint order:
%   [base shoulder elbow wrist pen]
%
% Coupled pair rule:
%   ID2 is the commanded shoulder motor
%   ID3 mirrors ID2 about the HOME relationship:
%       d2 = raw2 - rawHome2
%       d3 = -d2
%       raw3 = rawHome3 + d3

theta5 = theta5(:);
if numel(theta5) ~= 5
    error("theta5 must be 5x1.");
end

ids = double(cal.servoIDs(:));
isAX = logical(cal.isAX12(:));
rawHome = double(cal.rawHome(:));

if numel(ids) ~= 6 || numel(isAX) ~= 6 || numel(rawHome) ~= 6
    error("Calibration fields must all be length 6.");
end

% Keep this mapping explicit and simple:
% [base shoulder elbow wrist pen] -> [ID1 ID2 ID4 ID5 ID6]
moveitToServoID = [1; 2; 4; 5; 6];

if isfield(cal,"dirSignMoveIt") && numel(cal.dirSignMoveIt) == 5
    dirSignMoveIt = double(cal.dirSignMoveIt(:));
else
    dirSignMoveIt = [-1; +1; +1; -1; +1];
end

% counts per radian
cpr = zeros(6,1);
for i = 1:6
    if isAX(i)
        cpr(i) = 1023 / deg2rad(300);
    else
        cpr(i) = 4095 / deg2rad(360);
    end
end

raw6 = rawHome;

% Directly command IDs 1,2,4,5,6 from the 5-DOF vector
for j = 1:5
    id = moveitToServoID(j);
    idx = find(ids == id, 1);
    if isempty(idx)
        error("Servo ID %d not found in cal.servoIDs.", id);
    end
    raw6(idx) = rawHome(idx) + dirSignMoveIt(j) * theta5(j) * cpr(idx);
end

% Coupled pair: ID2 commanded directly, ID3 mirrors ID2 about HOME
idx2 = find(ids == 2, 1);
idx3 = find(ids == 3, 1);
if isempty(idx2) || isempty(idx3)
    error("Could not find IDs 2 and 3 in cal.servoIDs.");
end

d2 = raw6(idx2) - rawHome(idx2);
raw6(idx3) = rawHome(idx3) - d2;

raw6 = round(raw6(:));
end