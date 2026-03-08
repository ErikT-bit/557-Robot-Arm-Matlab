function hw = robot_hw_rb150_calibrated_6motor(port, baud)
% robot_hw_rb150_calibrated_6motor
%
% Board protocol:
%   Q                -> 12 bytes = 6x uint16 PRESENT_POSITION
%   G                -> 12 bytes = 6x uint16 GOAL_POSITION
%   R                -> 108 bytes = 6 records x 18 bytes each:
%                       [id u8]
%                       [torque_enable u8]
%                       [cw_angle_limit u16]
%                       [ccw_angle_limit u16]
%                       [moving_speed u16]
%                       [moving u8]
%                       [torque_limit u16]
%                       [torque_ctrl_mode_enable u8]
%                       [max_torque u16]
%                       [alarm_led u8]
%                       [shutdown u8]
%                       [present_voltage u8]
%                       [present_temperature u8]
%   J a b c d e f\n  -> set 6 raw GOAL_POSITION values
%   L a b c d e f\n  -> set 6 raw TORQUE_LIMIT values
%   T 1\n / T 0\n    -> torque on/off

cal = servo_calibration();

% Real pen vertical at -1.8 deg
% Model/viewer vertical at -9.1 deg
% model angle = measured angle - 7.3 deg
penModelOffsetRad = deg2rad(-7.3);

s = serialport(port, baud, "Timeout", 0.30);
configureTerminator(s, "LF");
flush(s);
pause(0.10);

try
    drain_text_lines_quick();
catch
end

hw.readMotors          = @readMotors;
hw.readGoalMotors      = @readGoalMotors;
hw.readMotorRegisters  = @readMotorRegisters;
hw.readJoints          = @readJoints;
hw.sendJoints          = @sendJoints;
hw.sendRawMotors       = @sendRawMotors;
hw.sendTorqueLimits    = @sendTorqueLimits;
hw.torqueOn            = @() torqueCmd(true);
hw.torqueOff           = @() torqueCmd(false);
hw.close               = @closePort;

    function raw6 = readMotors()
        clear_input_only();
        write(s, uint8('Q'), "uint8");
        raw = read_exact_bytes(12);
        raw6 = typecast(uint8(raw), "uint16");
        raw6 = double(raw6(:));
    end

    function raw6 = readGoalMotors()
        clear_input_only();
        write(s, uint8('G'), "uint8");
        raw = read_exact_bytes(12);
        raw6 = typecast(uint8(raw), "uint16");
        raw6 = double(raw6(:));
    end

    function regs = readMotorRegisters()
        clear_input_only();
        write(s, uint8('R'), "uint8");

        % 6 motors * 18 bytes each = 108 bytes
        raw = uint8(read_exact_bytes(108));

        regs = repmat(struct( ...
            'id', 0, ...
            'torque_enable', 0, ...
            'cw_angle_limit', 0, ...
            'ccw_angle_limit', 0, ...
            'moving_speed', 0, ...
            'moving', 0, ...
            'torque_limit', 0, ...
            'torque_ctrl_mode_enable', 0, ...
            'max_torque', 0, ...
            'alarm_led', 0, ...
            'shutdown', 0, ...
            'present_voltage', 0, ...
            'present_temperature', 0), 6, 1);

        idx = 1;
        for k = 1:6
            regs(k).id = double(raw(idx)); idx = idx + 1;
            regs(k).torque_enable = double(raw(idx)); idx = idx + 1;

            regs(k).cw_angle_limit = double(typecast(raw(idx:idx+1), 'uint16')); idx = idx + 2;
            regs(k).ccw_angle_limit = double(typecast(raw(idx:idx+1), 'uint16')); idx = idx + 2;
            regs(k).moving_speed = double(typecast(raw(idx:idx+1), 'uint16')); idx = idx + 2;

            regs(k).moving = double(raw(idx)); idx = idx + 1;

            regs(k).torque_limit = double(typecast(raw(idx:idx+1), 'uint16')); idx = idx + 2;

            regs(k).torque_ctrl_mode_enable = double(raw(idx)); idx = idx + 1;

            regs(k).max_torque = double(typecast(raw(idx:idx+1), 'uint16')); idx = idx + 2;

            regs(k).alarm_led = double(raw(idx)); idx = idx + 1;
            regs(k).shutdown = double(raw(idx)); idx = idx + 1;
            regs(k).present_voltage = double(raw(idx)); idx = idx + 1;
            regs(k).present_temperature = double(raw(idx)); idx = idx + 1;
        end
    end

    function theta5 = readJoints()
        raw6 = readMotors();
        theta5 = servo_to_moveit_rad(raw6, cal);
        theta5 = apply_model_offsets_read(theta5);
    end

    function sendJoints(theta5_model)
        theta5_servo = apply_model_offsets_write(theta5_model);
        raw6 = moveit_rad_to_servo(theta5_servo, cal);
        sendRawMotors(raw6);
    end

    function sendRawMotors(raw6)
        raw6 = double(raw6(:));
        if numel(raw6) ~= 6
            error("sendRawMotors expected 6 values.");
        end

        cmd = sprintf('J %u %u %u %u %u %u\n', ...
            round(raw6(1)), round(raw6(2)), round(raw6(3)), ...
            round(raw6(4)), round(raw6(5)), round(raw6(6)));

        clear_input_only();
        write(s, uint8(cmd), "uint8");
    end

    function sendTorqueLimits(raw6)
        raw6 = double(raw6(:));
        if numel(raw6) ~= 6
            error("sendTorqueLimits expected 6 values.");
        end

        cmd = sprintf('L %u %u %u %u %u %u\n', ...
            round(raw6(1)), round(raw6(2)), round(raw6(3)), ...
            round(raw6(4)), round(raw6(5)), round(raw6(6)));

        clear_input_only();
        write(s, uint8(cmd), "uint8");
    end

    function theta5m = apply_model_offsets_read(theta5s)
        theta5m = theta5s(:);
        if numel(theta5m) >= 5
            theta5m(5) = theta5m(5) + penModelOffsetRad;
        end
    end

    function theta5s = apply_model_offsets_write(theta5m)
        theta5s = theta5m(:);
        if numel(theta5s) >= 5
            theta5s(5) = theta5s(5) - penModelOffsetRad;
        end
    end

    function torqueCmd(on)
        clear_input_only();

        if on
            cmd = sprintf('T 1\n');
        else
            cmd = sprintf('T 0\n');
        end

        write(s, uint8(cmd), "uint8");

        t0 = tic;
        while toc(t0) < 0.30
            if s.NumBytesAvailable > 0
                try
                    txt = strtrim(readline(s));
                    if ~isempty(txt)
                        disp(txt);
                    end
                catch
                end
                break
            end
            pause(0.01);
        end
    end

    function raw = read_exact_bytes(nBytes)
        raw = zeros(1, nBytes, 'uint8');
        got = 0;
        t0 = tic;

        while got < nBytes
            if toc(t0) > s.Timeout + 0.25
                error("Serial read timeout: got %d of %d bytes.", got, nBytes);
            end

            avail = s.NumBytesAvailable;
            if avail > 0
                chunk = read(s, min(avail, nBytes - got), "uint8");
                raw(got + 1 : got + numel(chunk)) = uint8(chunk);
                got = got + numel(chunk);
            else
                pause(0.002);
            end
        end
    end

    function clear_input_only()
        t0 = tic;
        while toc(t0) < 0.03
            if s.NumBytesAvailable <= 0
                break
            end
            try
                read(s, min(s.NumBytesAvailable, 256), "uint8");
            catch
                break
            end
        end
    end

    function drain_text_lines_quick()
        t0 = tic;
        while toc(t0) < 0.20
            if s.NumBytesAvailable <= 0
                break
            end
            try
                readline(s);
            catch
                try
                    read(s, min(s.NumBytesAvailable, 256), "uint8");
                catch
                    break
                end
            end
        end
    end

    function closePort()
        try
            clear_input_only();
        catch
        end
        clear s
    end
end