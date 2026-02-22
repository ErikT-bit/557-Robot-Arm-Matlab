function cal = servo_calibration()
% SERVO_CALIBRATION  Hardware calibration data from the Arduino firmware.
%
% This file contains ALL the information needed to convert between
% MoveIt joint radians (used by your MATLAB IK) and physical Dynamixel
% servo commands.  Values are taken from the Arduino .ino files in the
% Otis-Spunkmeyer/ME_557_Robot repository (consistent across offline/
% online/live variants).
%
% COORDINATE SYSTEMS:
%   MoveIt radians : zero-centered, home = 0 rad (arm straight up)
%   Logical degrees: home = 180 deg (arm straight up), range varies per motor
%   Physical degrees: raw servo angle after applying physicalOffset
%   Raw bits       : AX-12 → 0-1023 (300 deg), MX/XM → 0-4095 (360 deg)
%
% USAGE:
%   cal = servo_calibration();
%   [goalPos, ids] = moveit_rad_to_servo(theta_rad, cal);

% --- Per-servo hardware configuration ---
%   Columns: ID, physicalOffset, logicalLower, logicalUpper, isAX12
%   "Logical" means the user-facing angle system centered at 180 deg.
cal.servo = struct( ...
    'id',             {1,     2,     3,     4,     5,     6    }, ...
    'physicalOffset', {117.0, 187.0, 189.0, 150.0, 150.0, 150.0}, ...
    'logicalLower',   {120.0, 120.0, 120.0,  40.0, 100.0, 140.0}, ...
    'logicalUpper',   {220.0, 240.0, 240.0, 240.0, 260.0, 300.0}, ...
    'isAX12',         {false, false, false,  true,  true,  true } ...
);

% --- MoveIt joint → Servo ID mapping ---
% 5 MoveIt joints map to 6 physical servos (Motor 3 mirrors Motor 2)
cal.moveitJointNames = ["Motor1_joint", "Motor2_L", "Motor4_elb", ...
                        "Motor5_wr", "Joint_EE"];
cal.moveitToServoID  = [1, 2, 4, 5, 6];   % index i → servo ID

% --- Direction signs ---
% Converts MoveIt positive rotation → logical degree direction.
%   +1 means MoveIt positive → logical increases (above 180)
%   -1 means MoveIt positive → logical decreases (below 180)
cal.dirSign = [-1, +1, +1, -1, +1];

% --- MoveIt home offsets (radians at logical 180 deg) ---
% Currently all zero — meaning MoveIt 0 rad = logical 180 deg exactly.
cal.homeRad = [0.0, 0.0, 0.0, 0.0, 0.0];

% --- Mirror joint ---
% Motor 3 (ID=3) mirrors Motor 2 (ID=2): logical3 = 360 - logical2
cal.mirrorServoID   = 3;
cal.mirrorSourceID  = 2;

% --- Motor 5 safety tuck ---
% When Motor 4 logical angle < 70 deg, Motor 5 must tuck to 100 deg
% to avoid physical collision between wrist and elbow.
cal.motor5TuckThreshold = 70.0;   % Motor 4 logical deg
cal.motor5TuckAngle     = 100.0;  % Motor 5 logical deg when tucked

% --- Communication ---
cal.baudRate = 1000000;
cal.protocolVersion = 1.0;

% --- Pen servo (PWM, not Dynamixel) ---
% Controlled via Arduino pin 3, addressed as "ID 7" in the online sketch.
% Range 0-180 deg, home at 90 deg.  Not part of the MoveIt kinematic chain.
cal.penServoPin  = 3;
cal.penServoHome = 90;  % degrees
end
