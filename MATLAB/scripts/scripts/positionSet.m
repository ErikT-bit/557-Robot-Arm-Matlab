function position = positionSet(s, goalPos)
% Send 2 bytes (low/high) and read 2 bytes back (low/high)
% s = serialport(...)
% goalPos = 0..1023

flush(s);

goalPos = uint16(goalPos);
low  = uint8(bitand(goalPos,255));
high = uint8(bitshift(goalPos,-8));

write(s, low,  "uint8");
write(s, high, "uint8");

lo = read(s, 1, "uint8");
hi = read(s, 1, "uint8");
position = double(lo) + 256*double(hi);
end