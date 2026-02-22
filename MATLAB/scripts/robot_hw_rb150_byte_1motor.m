function hw = robot_hw_rb150_byte_1motor(port, baud)
% RB150 byte protocol (your current Arduino sketch):
% MATLAB sends:  lowByte, highByte  (goalPos 0..1023)
% RB150 replies: lowByte, highByte  (presentPos 0..1023)

s = serialport(port, baud, "Timeout", 2);
flush(s);

hw.sendGoal = @sendGoal;
hw.close = @closePort;

    function pos = sendGoal(goalPos)
        goalPos = uint16(max(0, min(1023, round(goalPos))));
        low  = uint8(bitand(goalPos, 255));
        high = uint8(bitshift(goalPos, -8));

        flush(s);
        write(s, low,  "uint8");
        write(s, high, "uint8");

        lo = read(s, 1, "uint8");
        hi = read(s, 1, "uint8");
        pos = double(lo) + 256*double(hi);
    end

    function closePort()
        clear s
    end
end