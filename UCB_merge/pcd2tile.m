function table = pcd2tile(maps,filename)
    table = gen_tile(maps);
    save_tile(table, ['tiles/',filename]);
end

