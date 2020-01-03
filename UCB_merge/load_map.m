function map=load_map(path)
    pc_path = [path, '/pc.pcd'];
    gps_path = [path, '/gps.txt'];
    map.gps_origin = load(gps_path);
    map.pc = pcread(pc_path);
end