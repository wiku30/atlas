function maps = gen_maps(path,n)
    for i=1:n
        pathi = [path, num2str(i)];
        maps(i)= gen_map(pathi);
    end
    for i=1:n
        maps(i) = change_origin(maps(i),maps(1).gps_origin);
    end
end