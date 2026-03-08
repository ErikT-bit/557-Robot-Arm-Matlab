function reestablish_true_home()
hw = robot_hw_rb150_raw6motor("COM13",1000000);

disp("Torque OFF...");
hw.torqueOff();
pause(0.3);

disp("Manually place the robot in TRUE PHYSICAL HOME.");
disp("When it is exactly where you want home to be, press ENTER.");
input("", "s");

rawHome = double(hw.readMotors());

fprintf("\nTRUE HOME raw counts:\n");
for i = 1:6
    fprintf("  ID%d: %d\n", i, round(rawHome(i)));
end

assignin('base','rawHome_true',rawHome);
disp("Saved to workspace variable: rawHome_true");
end