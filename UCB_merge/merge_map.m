function mapout = merge_map(map_base, map_add, varargin) % 1: direct merge
    origin = map_base.gps_origin;
    map_add = change_origin(map_add, map_base.gps_origin);
    resolution = 1.2;
    ndt_size = 6;
    if nargin>=3
        if varargin{1} <= 0
            mapout = direct_merge(map_base, map_add.pc);
            return;
        else
            resolution = varargin{1};
        end
        if nargin>=4
            ndt_size = varargin{2};
        end
    end
    pc1 = pcdownsample(map_base.pc,'gridAverage',resolution);
    pc2 = pcdownsample(map_add.pc,'gridAverage',resolution);
    [tf,reged,err] = pcregisterndt(pc2,pc1,ndt_size,'InitialTransform',affine3d(eye(4)),'Verbose',true,'MaxIterations',100);
    mapout = direct_merge(map_base,pctransform(map_add.pc,tf));
end

function mapout = direct_merge(map1,pc2)
    mapout.gps_origin = map1.gps_origin;
    mapout.pc = pcmerge(map1.pc,pc2,0.1);
end