function [map,err] = gen_map(name,varargin)
    gps = load([name,'T_gps.txt']);
    rel = load([name,'_rel.txt']);
    pcd = pcread([name,'.pcd']);
    if nargin>=4
        [map,err] = frame_init(pcd, gps, rel, varargin);
    else
        [map,err] = frame_init(pcd, gps, rel);
    end
end