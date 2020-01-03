function [x,y]=i2xy(i);
    x = int32(floor(double(i)/32768)-16384);
    y = int32(mod(i,32768)-16384);
end