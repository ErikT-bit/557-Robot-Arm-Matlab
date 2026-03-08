function[position] = positionSet(serial, goalPos)
    flush(serial)
    low = mod(goalPos, 256);

    high = floor(goalPos/256);

    fwrite(serial, low);
    fwrite(serial, high);

    position = fread(serial,1) + fread(serial,1)*256;

end
%Save that, then before running type 

s = serialport%('Your COM chanel here', 9600)
fopen(s)

%Then you can input your desired positions
%finally, before changing any arduino code and uploading you need to disconnect matlab by typing 
clear s