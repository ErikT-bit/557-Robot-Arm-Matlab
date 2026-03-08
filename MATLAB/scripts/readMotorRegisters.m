function regs = readMotorRegisters()
    clear_input_only();
    write(s, uint8('R'), "uint8");

    % 6 motors * 12 bytes each = 72 bytes
    raw = uint8(read_exact_bytes(72));

    regs = repmat(struct( ...
        'id', 0, ...
        'torque_enable', 0, ...
        'cw_angle_limit', 0, ...
        'ccw_angle_limit', 0, ...
        'moving_speed', 0, ...
        'moving', 0, ...
        'torque_limit', 0, ...
        'torque_control_mode_enable', 0), 6, 1);

    idx = 1;
    for k = 1:6
        regs(k).id = double(raw(idx)); idx = idx + 1;
        regs(k).torque_enable = double(raw(idx)); idx = idx + 1;

        regs(k).cw_angle_limit  = double(typecast(raw(idx:idx+1), 'uint16')); idx = idx + 2;
        regs(k).ccw_angle_limit = double(typecast(raw(idx:idx+1), 'uint16')); idx = idx + 2;
        regs(k).moving_speed    = double(typecast(raw(idx:idx+1), 'uint16')); idx = idx + 2;

        regs(k).moving = double(raw(idx)); idx = idx + 1;

        regs(k).torque_limit = double(typecast(raw(idx:idx+1), 'uint16')); idx = idx + 2;

        regs(k).torque_control_mode_enable = double(raw(idx)); idx = idx + 1;
    end
end