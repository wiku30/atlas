function save_map(map,path)
    pcwrite(map.pc, [path,'/pc.pcd'],'Encoding','binary');
    gps=map.gps_origin
    save([path,'/gps.txt'],'gps','-ascii','-double');
end