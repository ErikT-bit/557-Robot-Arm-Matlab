function save_true_home_only()
port = "COM13";
baud = 1000000;

calDir  = fullfile(pwd,"calibration");
matPath = fullfile(calDir,"servo_cal_user.mat");

if ~exist(matPath,"file")
    error("No calibration file found: %s", matPath);
end

S = load(matPath,"cal");
cal = S.cal;

hw = robot_hw_rb150_raw6motor(port, baud);

disp("Torque OFF...");
hw.torqueOff();
pause(0.3);

disp("Place robot at TRUE PHYSICAL HOME, then press ENTER.");
input("", "s");

cal.rawHome = double(hw.readMotors());

fprintf("\nUpdated rawHome:\n");
for i = 1:6
    fprintf("  ID%d: %d\n", i, round(cal.rawHome(i)));
end

save(matPath,"cal");

mPath = fullfile(calDir,"servo_cal_user.m");
write_cal_function(mPath, cal);

fprintf("\nUpdated:\n  %s\n  %s\n", matPath, mPath);
end

function write_cal_function(path, cal)
fid = fopen(path, "w");
fprintf(fid, "function cal = servo_cal_user()\n");
fprintf(fid, "%% Auto-generated on %s\n\n", datestr(now));

fprintf(fid, "cal.createdAt = '%s';\n", datestr(now));
fprintf(fid, "cal.port = '%s';\n", cal.port);
fprintf(fid, "cal.baud = %d;\n", cal.baud);

fprintf(fid, "cal.servoIDs = [%s]'';\n", numlist(cal.servoIDs));
fprintf(fid, "cal.isAX12   = [%s]'';\n", numlist(double(cal.isAX12)));

fprintf(fid, "cal.rawMin  = [%s]'';\n", numlist(cal.rawMin));
fprintf(fid, "cal.rawMax  = [%s]'';\n", numlist(cal.rawMax));
fprintf(fid, "cal.rawHome = [%s]'';\n", numlist(cal.rawHome));

fprintf(fid, "cal.moveitToServoID = [%s]'';\n", numlist(cal.moveitToServoID));
fprintf(fid, "cal.dirSignMoveIt   = [%s]'';\n", numlist(cal.dirSignMoveIt));

if isfield(cal,'coupledPair')
    fprintf(fid, "cal.coupledPair = [%s]'';\n", numlist(cal.coupledPair));
else
    fprintf(fid, "cal.coupledPair = [2 3]'';\n");
end

fprintf(fid, "end\n");
fclose(fid);
end

function s = numlist(v)
v = double(v(:))';
s = sprintf("%.0f ", v);
s = strtrim(s);
end