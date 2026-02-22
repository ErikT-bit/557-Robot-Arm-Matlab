function hw = robot_hw_rb150_raw6motor(port, baud)
% robot_hw_rb150_raw6motor
% Minimal RB150 interface:
%   'Q'   -> reads 12 bytes = 6x uint16 (little-endian)
%   'T 1' -> torque on   (expects a text response line)
%   'T 0' -> torque off  (expects a text response line)

s = serialport(port, baud, "Timeout", 2);
flush(s);

hw.readMotors = @readMotors;
hw.torqueOn   = @() torqueCmd(true);
hw.torqueOff  = @() torqueCmd(false);
hw.close      = @closePort;

    function pos6 = readMotors()
        flush(s);
        write(s, uint8('Q'), "uint8");
        raw = read(s, 12, "uint8");
        pos6 = typecast(uint8(raw), "uint16");
        pos6 = pos6(:);
    end

    function torqueCmd(on)
        flush(s);
        configureTerminator(s, "LF");
        if on
            write(s, uint8(['T',' ','1']), "uint8");
        else
            write(s, uint8(['T',' ','0']), "uint8");
        end
        % Try to read back the "OK ..." line (if firmware prints it)
        try
            msg = readline(s);
            disp(strtrim(msg));
        catch
        end
    end

    function closePort()
        % Properly release the serialport object
        try
            flush(s);
        catch
        end
        clear s
    end
end
