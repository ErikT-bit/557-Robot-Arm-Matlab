function hw = robot_hw_rb150_raw6motor(port, baud)
% robot_hw_rb150_raw6motor
% Minimal RB150 interface:
%   'Q' -> reads 12 bytes = 6x uint16 (LE)
%   'T 1' / 'T 0' -> reads a line like "OK TON"/"OK TOFF" (if firmware prints)

s = serialport(port, baud, "Timeout", 2);
flush(s);

hw.readMotors = @readMotors;
hw.torqueOn   = @() torqueCmd(true);
hw.torqueOff  = @() torqueCmd(false);
hw.close      = @() clear s;

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
        try
            msg = readline(s);
            disp(strtrim(msg));
        catch
        end
    end
end