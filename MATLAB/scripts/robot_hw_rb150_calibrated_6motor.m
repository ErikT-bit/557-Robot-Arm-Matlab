function hw = robot_hw_rb150_calibrated_6motor(port, baud)
% robot_hw_rb150_calibrated_6motor
% Robust RB150 interface (does NOT require "OK" responses to J commands).
%
% Expected firmware protocol:
%   Q                -> returns 12 bytes = 6x uint16 (little-endian)
%   J a b c d e f\n  -> sets goals (may or may not print anything)
%   T 1\n / T 0\n    -> torque on/off (may print "OK TON"/"OK TOFF" or nothing)

cal = servo_calibration();

s = serialport(port, baud, "Timeout", 0.2);   % short timeout, avoid hanging
configureTerminator(s, "LF");
flush(s);

% Optional: read one startup line if present (don’t block)
try
    if s.NumBytesAvailable > 0
        disp(strtrim(readline(s)));
    end
catch
end

hw.readMotors = @readMotors;
hw.readJoints = @readJoints;
hw.sendJoints = @sendJoints;
hw.torqueOn   = @() torqueCmd(true);
hw.torqueOff  = @() torqueCmd(false);
hw.close      = @closePort;

    function raw6 = readMotors()
        flush(s);
        write(s, uint8('Q'), "uint8");
        raw = read(s, 12, "uint8");
        if numel(raw) ~= 12
            error("RB150 Q read returned %d bytes (expected 12).", numel(raw));
        end
        raw6 = typecast(uint8(raw), "uint16");
        raw6 = double(raw6(:));
    end

    function theta5 = readJoints()
        raw6 = readMotors();
        theta5 = servo_to_moveit_rad(raw6, cal);
    end

    function sendJoints(theta5)
        raw6 = moveit_rad_to_servo(theta5, cal);

        cmd = sprintf('J %d %d %d %d %d %d\n', ...
            round(raw6(1)), round(raw6(2)), round(raw6(3)), ...
            round(raw6(4)), round(raw6(5)), round(raw6(6)));

        % Clear any pending bytes so we don’t accidentally read old stuff later
        flush(s);

        % Send command; do NOT wait for response
        write(s, uint8(cmd), "uint8");

        % Optional: if firmware prints responses sometimes, drain quickly (non-blocking)
        drain_lines_quick();
    end

    function torqueCmd(on)
        flush(s);
        if on
            cmd = sprintf('T 1\n');
        else
            cmd = sprintf('T 0\n');
        end
        write(s, uint8(cmd), "uint8");

        % Try to read a response line if it exists, but don’t hang
        t0 = tic;
        while toc(t0) < 0.2
            if s.NumBytesAvailable > 0
                try
                    disp(strtrim(readline(s)));
                catch
                end
                break
            end
            pause(0.01);
        end
    end

    function drain_lines_quick()
        % Drain any text quickly without blocking
        t0 = tic;
        while toc(t0) < 0.05
            if s.NumBytesAvailable <= 0, break; end
            try
                readline(s); %#ok<NASGU>
            catch
                break
            end
        end
    end

    function closePort()
        try, flush(s); catch, end
        clear s
    end
end
