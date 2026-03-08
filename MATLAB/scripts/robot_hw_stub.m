function hw = robot_hw_stub(robot)
% Stubbed hardware interface:
% - readJoints() returns last commanded theta
% - sendJoints(theta) updates internal state
%
% Replace these with your real motor comms later.

state.theta = robot.home;

hw.readJoints = @readJoints;
hw.sendJoints = @sendJoints;

    function th = readJoints()
        th = state.theta;
    end

    function sendJoints(th)
        state.theta = th;
        % For debugging you can uncomment:
        % fprintf("sendJoints: [%s]\n", sprintf("%.3f ", th));
    end
end