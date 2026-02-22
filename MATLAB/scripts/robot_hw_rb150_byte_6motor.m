function hw = robot_hw_rb150_byte_6motor(port, baud, map)
% RB150 6-motor byte bridge (Protocol 1.0) with mirror constraint:
% motors: IDs [1 2 3 4 5 6]
% constraint: motor2 mirrors motor3 ALWAYS (cannot be independent)

s = serialport(port, baud, "Timeout", 2);
flush(s);

% ---------- Defaults ----------
if nargin < 3 || isempty(map)
    map.ids = [1 2 3 4 5 6];

    % Motor position range assumptions:
    % AX-12A: 0..1023 over ~300deg
    % MX-64AT: commonly 0..4095 over 360deg (depends on configuration)
    %
    % We'll set per-motor ranges and scales (counts per rad).
    ax_countsPerRad = 1023 / (300*pi/180);
    mx_countsPerRad = 4095 / (2*pi);  % assume 0..4095 over 360deg

    map.min = [0; 0; 0; 0; 0; 0];
    map.max = [1023; 4095; 4095; 1023; 1023; 1023];

    map.mid = [512; 2048; 2048; 512; 512; 512];

    map.dir = [ +1; +1; +1; +1; +1; +1 ];

    map.scale = [ax_countsPerRad; mx_countsPerRad; mx_countsPerRad; ax_countsPerRad; ax_countsPerRad; ax_countsPerRad];

    % Mirror constraint between motor2 and motor3 in count domain:
    % goal2 = mirrorC - goal3
    map.mirrorC = 4095;  % set to 1023 if your MX are configured 0..1023
end

hw.sendJoints = @sendJoints;   % input: 5x1 joint radians
hw.readJoints = @readJoints;   % output: 5x1 joint radians
hw.readMotors = @readMotors;   % output: 6x1 motor counts
hw.torqueOn   = @() torqueCmd(true);
hw.torqueOff  = @() torqueCmd(false);
hw.close      = @closePort;

% ---------- Mapping ----------
% Joint vector is 5 DOF: [θ1 θ2 θ3 θ4 θ5]
% Motor vector is 6 motors: [m1 m2 m3 m4 m5 m6]
% We assume:
%   m1 <- θ1
%   m3 <- θ2   (and m2 mirrors m3)
%   m4 <- θ3
%   m5 <- θ4
%   m6 <- θ5
% If you later decide θ2/θ3 mapping differs, adjust only these lines.

    function gp = motorThetaToGoal(motorTheta6)
        motorTheta6 = motorTheta6(:);
        gp = map.mid(:) + map.dir(:) .* (map.scale(:) .* motorTheta6);
        gp = round(gp);
        gp = max(map.min(:), min(map.max(:), gp));
        gp = uint16(gp);
    end

    function motorTheta6 = motorGoalToTheta(gp)
        gp = double(gp(:));
        motorTheta6 = (gp - map.mid(:)) ./ (map.dir(:) .* map.scale(:));
    end

    function gp6 = joints5_to_goals6(theta5)
        theta5 = theta5(:);

        motorTheta6 = zeros(6,1);
        motorTheta6(1) = theta5(1); % m1
        motorTheta6(3) = theta5(2); % m3 (driven)
        motorTheta6(4) = theta5(3); % m4
        motorTheta6(5) = theta5(4); % m5
        motorTheta6(6) = theta5(5); % m6

        % convert to goals
        gp6 = motorThetaToGoal(motorTheta6);

        % enforce mirror in count domain
        gp6(2) = uint16(max(map.min(2), min(map.max(2), map.mirrorC - double(gp6(3)))));
    end

    function theta5 = goals6_to_joints5(gp6)
        gp6 = double(gp6(:));
        motorTheta6 = motorGoalToTheta(gp6);

        theta5 = zeros(5,1);
        theta5(1) = motorTheta6(1);
        theta5(2) = motorTheta6(3); % read from m3 (primary)
        theta5(3) = motorTheta6(4);
        theta5(4) = motorTheta6(5);
        theta5(5) = motorTheta6(6);
    end

% ---------- IO ----------
    function resp = sendRawGoals6(gp6)
        gp6 = uint16(gp6(:));
        flush(s);
        write(s, uint8('J'), "uint8");
        write(s, typecast(gp6, "uint8"), "uint8");   % 12 bytes

        raw = read(s, 12, "uint8");
        resp = typecast(uint8(raw), "uint16");
        resp = resp(:);
    end

    function gp6 = readMotors()
        flush(s);
        write(s, uint8('Q'), "uint8");
        raw = read(s, 12, "uint8");
        gp6 = typecast(uint8(raw), "uint16");
        gp6 = gp6(:);
    end

    function pos = sendJoints(theta5)
        gp6 = joints5_to_goals6(theta5);
        meas6 = sendRawGoals6(gp6);

        pos.goal6 = double(gp6);
        pos.meas6 = double(meas6);
        pos.theta5_meas = goals6_to_joints5(double(meas6));
    end

    function theta5 = readJoints()
        meas6 = double(readMotors());
        theta5 = goals6_to_joints5(meas6);
    end

    function torqueCmd(on)
        flush(s);
        if on
            write(s, uint8(['T',' ','1']), "uint8");
        else
            write(s, uint8(['T',' ','0']), "uint8");
        end
        try
            configureTerminator(s, "LF");
            msg = readline(s);
            disp(strtrim(msg));
        catch
        end
    end

    function closePort()
        clear s
    end
end