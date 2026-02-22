function hw = robot_hw_rb150_calibrated_6motor(port, baud)
% robot_hw_rb150_calibrated_6motor
% Uses RB150 firmware protocol:
%   send: 'J' + 12 bytes (6x uint16 LE)
%   recv: 12 bytes (6x uint16 LE)
%   'Q' -> 12 bytes
%   'T 1' / 'T 0' -> "OK TON"/"OK TOFF" line

cal = servo_calibration();

s = serialport(port, baud, "Timeout", 2);
flush(s);

hw.sendJoints = @sendJoints;   % input: 5x1 MoveIt radians
hw.readJoints = @readJoints;   % output: 5x1 MoveIt radians
hw.readMotors = @readMotors;   % output: 6x1 raw
hw.torqueOn   = @() torqueCmd(true);
hw.torqueOff  = @() torqueCmd(false);
hw.close      = @closePort;

    function pos = sendJoints(theta5)
        theta5 = theta5(:);

        [goals6, ~] = moveit_rad_to_servo(theta5, cal);
        meas6 = sendRawGoals6(goals6);

        pos.goal6 = double(goals6(:));
        pos.meas6 = double(meas6(:));
        pos.theta5_meas = servo_to_moveit_rad(double(meas6(:)), cal);
    end

    function theta5 = readJoints()
        meas6 = double(readMotors());
        theta5 = servo_to_moveit_rad(meas6, cal);
    end

    function meas6 = sendRawGoals6(goals6)
        goals6 = uint16(goals6(:));
        flush(s);
        write(s, uint8('J'), "uint8");
        write(s, typecast(goals6, "uint8"), "uint8");  % 12 bytes
        raw = read(s, 12, "uint8");
        meas6 = typecast(uint8(raw), "uint16");
        meas6 = meas6(:);
    end

    function meas6 = readMotors()
        flush(s);
        write(s, uint8('Q'), "uint8");
        raw = read(s, 12, "uint8");
        meas6 = typecast(uint8(raw), "uint16");
        meas6 = meas6(:);
    end

    function torqueCmd(on)
        flush(s);
        configureTerminator(s, "LF");
        if on
            write(s, uint8(['T',' ','1']), "uint8");
        else
            write(s, uint8(['T',' ','0']), "uint8");
        end
        try
            msg = readline(s);
            disp(strtrim(msg));
        catch
        end
    end

    function closePort()
        clear s
    end
end