function hw = robot_hw_rb150_byte_5motor(port, baud, jointMap)
% RB150 5-motor bridge:
% - send: 'J' + 10 bytes (5x uint16 goal positions LE)
% - recv: 10 bytes (5x uint16 present positions LE)
% - 'Q' returns 10 bytes positions
% - 'T 1' torque on, 'T 0' torque off
%
% jointMap fields:
%   .mid   (5x1) mid counts, default 512
%   .dir   (5x1) +1/-1 direction
%   .scale (5x1) counts per rad (AX: 1023 / 300deg)
%   .min   (5x1) min counts
%   .max   (5x1) max counts

s = serialport(port, baud, "Timeout", 2);
flush(s);

% defaults for AX position mode (0..1023 over ~300deg)
countsPerRad = 1023 / (300*pi/180);

if nargin < 3 || isempty(jointMap)
    jointMap.mid   = 512*ones(5,1);
    jointMap.dir   = ones(5,1);
    jointMap.scale = countsPerRad*ones(5,1);
    jointMap.min   = zeros(5,1);
    jointMap.max   = 1023*ones(5,1);
end

hw.sendJoints = @sendJoints;
hw.readJoints = @readJoints;
hw.torqueOn   = @() torqueCmd(true);
hw.torqueOff  = @() torqueCmd(false);
hw.close      = @closePort;

    function gp = thetaToGoal(theta)
        theta = theta(:);
        gp = jointMap.mid + jointMap.dir .* (jointMap.scale .* theta);
        gp = round(gp);
        gp = max(jointMap.min, min(jointMap.max, gp));
        gp = uint16(gp);
    end

    function theta = goalToTheta(gp)
        gp = double(gp(:));
        theta = (gp - jointMap.mid) ./ (jointMap.dir .* jointMap.scale);
    end

    function pos = sendJoints(theta)
        gp = thetaToGoal(theta);

        flush(s);
        write(s, uint8('J'), "uint8");
        write(s, typecast(gp, "uint8"), "uint8");   % little-endian on Windows/MATLAB

        % read 10 bytes back = 5 uint16
        raw = read(s, 2*5, "uint8");
        gp_meas = typecast(uint8(raw), "uint16");
        pos.goal = double(gp);
        pos.meas = double(gp_meas(:));
        pos.theta_meas = goalToTheta(double(gp_meas(:)));
    end

    function theta = readJoints()
        flush(s);
        write(s, uint8('Q'), "uint8");
        raw = read(s, 2*5, "uint8");
        gp_meas = typecast(uint8(raw), "uint16");
        theta = goalToTheta(double(gp_meas(:)));
    end

    function torqueCmd(on)
        flush(s);
        if on
            write(s, uint8(['T',' ','1']), "uint8");
        else
            write(s, uint8(['T',' ','0']), "uint8");
        end
        % read the "OK ..." line
        try
            configureTerminator(s, "LF");
            msg = readline(s);
            disp(strtrim(msg));
        catch
            % ignore if line not received
        end
    end

    function closePort()
        clear s
    end
end