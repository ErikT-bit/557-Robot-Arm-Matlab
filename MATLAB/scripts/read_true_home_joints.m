function read_true_home_joints()
hw = robot_hw_rb150_calibrated_6motor("COM13",1000000);

disp("Torque ON...");
hw.torqueOn();
pause(0.5);

disp("Place robot at TRUE HOME and press ENTER.");
input("", "s");

theta_home = hw.readJoints();

fprintf("theta_home_deg = [%s]\n", sprintf('%+.1f  ', rad2deg(theta_home)));
assignin('base','theta_home_true',theta_home);
end