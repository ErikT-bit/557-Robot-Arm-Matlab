function theta5 = servo_to_moveit_rad(raw6, cal)
% servo_to_moveit_rad
% Converts 6 servo raw positions -> 5 MoveIt joint angles (rad).
% Uses cal.moveitToServoID and cal.dirSignMoveIt.
% Handles coupled pair metadata but will still produce angles even if home is stale.

raw6 = double(raw6(:));
if numel(raw6) ~= 6
    error("raw6 must be 6x1.");
end

ids = double(cal.servoIDs(:));
if numel(ids) ~= 6
    error("cal.servoIDs must be 6x1.");
end

if ~isfield(cal,"moveitToServoID") || numel(cal.moveitToServoID) ~= 5
    moveitToServoID = [1;2;4;5;6];
else
    moveitToServoID = double(cal.moveitToServoID(:));
end

if ~isfield(cal,"dirSignMoveIt") || numel(cal.dirSignMoveIt) ~= 5
    dirSignMoveIt = [-1; +1; +1; -1; +1];
else
    dirSignMoveIt = double(cal.dirSignMoveIt(:));
end

rawHome = double(cal.rawHome(:));
if numel(rawHome) ~= 6 || any(isnan(rawHome))
    rawHome = (double(cal.rawMin(:)) + double(cal.rawMax(:))) / 2;
end

isAX = logical(cal.isAX12(:));
cpr = zeros(6,1);
cpr(isAX)  = 1023 / deg2rad(300);
cpr(~isAX) = 4095 / deg2rad(360);

theta5 = zeros(5,1);
for j = 1:5
    id = moveitToServoID(j);
    idx = find(ids == id, 1);
    if isempty(idx), error("moveitToServoID contains ID%d not in servoIDs.", id); end

    theta5(j) = (raw6(idx) - rawHome(idx)) / cpr(idx);
    theta5(j) = theta5(j) * dirSignMoveIt(j);
end
end