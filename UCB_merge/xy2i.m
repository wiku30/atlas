function i=xy2i(x,y)
    i = int32(floor((x+16384)*32768 + (y+16384)));
end