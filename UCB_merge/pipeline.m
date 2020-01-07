function pipeline(series, number)
    maps = gen_maps(series, number);
    pcd2tile(maps, series);
    title = 'maps_unified/'
    for i=1:number
        mkdir([title ,series,num2str(i)]);
        save_map(maps(i),[title,series,num2str(i)]);
    end
    [utm(1) utm(2) xx] = deg2utm(maps(1).gps_origin(1),maps(1).gps_origin(2));
    utm(3) = maps(1).gps_origin(3);
    save([title,'utm.txt'],'utm','-ascii','-double');
end