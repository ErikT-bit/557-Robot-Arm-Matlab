function cal = servo_cal_user()
% Auto-generated on 05-Mar-2026 15:21:29

cal.createdAt = '05-Mar-2026 15:21:29';
cal.port = 'COM13';
cal.baud = 1000000;
cal.servoIDs = [1 2 3 4 5 6]'';
cal.isAX12   = [1 0 0 1 1 1]'';
cal.rawMin  = [424 1099 1831 175 328 362]'';
cal.rawMax  = [664 2265 2998 588 727 596]'';
cal.rawHome = [510 1965 2132 510 506 508]'';
cal.moveitToServoID = [1 2 4 5 6]'';
cal.dirSignMoveIt   = [-1 1 1 -1 1]'';
cal.coupledPair = [2 3]'';
end
