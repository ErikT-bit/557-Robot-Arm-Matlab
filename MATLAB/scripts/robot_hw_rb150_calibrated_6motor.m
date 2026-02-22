function hw = robot_hw_rb150_calibrated_6motor(port, baud)
% robot_hw_rb150_calibrated_6motor
% High-level hardware wrapper using user calibration.
% Exposes:
%   hw.readMotors() -> raw6
%   hw.readJoints() -> theta5 (MoveIt)
%   hw.sendJoints(theta5)
%   hw.torqueOn(), hw.torqueOff()

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

        % Send as ASCII command: J r1 r2 r3 r4 r5 r6\n
        cmd = sprintf("J %d %d %d %d %d %d\n", round(raw6(1)), round(raw6(2)), round(raw6(3)), round(raw6(4)), round(raw6(5)), round(raw6(6)));
        write(s, uint8(cmd), "uint8");

        % optional response line
        try
            msg = readline(s);
            disp(strtrim(msg));
        catch
        end
    end

    function torqueCmd(on)
        flush(s);
        if on
            write(s, uint8("T 1"+newline), "uint8");
        else
            write(s, uint8("T 0"+newline), "uint8");
        end
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
