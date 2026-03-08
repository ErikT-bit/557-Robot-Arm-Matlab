function test_coupled_23_pair()
clc;

port = "COM13";
baud = 1000000;

hw = robot_hw_rb150_calibrated_6motor(port, baud);

disp("===========================================");
disp("COUPLED MOTOR 2-3 MIRRORED TEST");
disp("This test moves motors 2 and 3 together only.");
disp("Motor 2 gets +d, motor 3 gets -d (or vice versa).");
disp("===========================================");

try
    hw.torqueOn();
    pause(0.5);

    raw0 = hw.readMotors();
    theta0 = hw.readJoints();

    fprintf("\nInitial raw counts:\n");
    disp(raw0.');

    fprintf("Initial joints (deg):\n");
    disp(rad2deg(theta0(:)).');

    d = input('Enter small raw-count step size [recommended 10 to 20, default 15]: ');
    if isempty(d)
        d = 15;
    end

    sgn = input('Enter mirrored direction sign (+1 or -1) [default +1]: ');
    if isempty(sgn)
        sgn = 1;
    end
    if ~(sgn == 1 || sgn == -1)
        error('Direction sign must be +1 or -1.');
    end

    fprintf("\nAbout to command:\n");
    fprintf("  motor2 = motor2 %+d counts\n", sgn*d);
    fprintf("  motor3 = motor3 %+d counts\n", -sgn*d);

    resp = input('Press ENTER to continue or type q to abort: ','s');
    if strcmpi(strtrim(resp),'q')
        disp("Aborted.");
        return
    end

    raw1 = raw0;
    raw1(2) = raw0(2) + sgn*d;
    raw1(3) = raw0(3) - sgn*d;

    fprintf("\nSending mirrored pair command...\n");
    hw.sendRawMotors(raw1);
    pause(1.0);

    raw_after = hw.readMotors();
    theta_after = hw.readJoints();

    fprintf("Raw after move:\n");
    disp(raw_after.');

    fprintf("Delta raw after move:\n");
    disp((raw_after - raw0).');

    fprintf("Joints after move (deg):\n");
    disp(rad2deg(theta_after(:)).');

    fprintf("Delta joints after move (deg):\n");
    disp(rad2deg(theta_after(:) - theta0(:)).');

    fprintf("\nReturning to original raw position...\n");
    hw.sendRawMotors(raw0);
    pause(1.0);

    raw_back = hw.readMotors();
    theta_back = hw.readJoints();

    fprintf("Raw after return:\n");
    disp(raw_back.');

    fprintf("Delta raw from original after return:\n");
    disp((raw_back - raw0).');

    fprintf("Joints after return (deg):\n");
    disp(rad2deg(theta_back(:)).');

    fprintf("Delta joints from original after return (deg):\n");
    disp(rad2deg(theta_back(:) - theta0(:)).');

    disp("===========================================");
    disp("HOW TO INTERPRET:");
    disp("If motors 2 and 3 are working, raw(2) and raw(3) should both change");
    disp("with opposite signs and similar magnitude.");
    disp("If one does not change, that motor/firmware path is broken.");
    disp("If neither changes, the J-command path is broken for that pair.");
    disp("===========================================");

catch ME
    fprintf(2,"\nTEST FAILED:\n%s\n", ME.message);
end
end