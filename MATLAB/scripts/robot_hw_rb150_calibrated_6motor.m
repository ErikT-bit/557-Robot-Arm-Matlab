function hw = robot_hw_rb150_calibrated_6motor(port, baud)
% robot_hw_rb150_calibrated_6motor
% High-level RB150 interface using user calibration.
% Commands expected by RB150 firmware:
%   Q            -> returns 12 bytes (6 x uint16 LE)
%   J a b c d e f\n  -> set 6 goal positions (raw counts)
%   T 1\n        -> torque ON
%   T 0\n        -> torque OFF

cal = servo_calibration();

s = serialport(port, baud, "Timeout", 2);
configureTerminator(s, "LF");
flush(s);

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

        flush(s);
        write(s, uint8(cmd), "uint8");

        % optional response line "OK ..."
        try
            msg = readline(s);
            disp(strtrim(msg));
        catch
        end
    end

    function torqueCmd(on)
        flush(s);

        if on
            cmd = sprintf('T 1\n');
        else
            cmd = sprintf('T 0\n');
        end

        write(s, uint8(cmd), "uint8");

        % optional response
        try
            msg = readline(s);
            disp(strtrim(msg));
        catch
        end
    end

    function closePort()
        try, flush(s); catch, end
        clear s
    end
end
