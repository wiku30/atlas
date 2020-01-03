function mapout = change_origin(mapin,new_gps_origin)
    gps0 = mapin.gps_origin;
    gps1 = new_gps_origin;
    [enu_trans(1),enu_trans(2),enu_trans(3)] = geodetic2enu(gps1(1),gps1(2),gps1(3),gps0(1),gps0(2),gps0(3),wgs84Ellipsoid);
    tf = eye(4); % Assume ground is a plane, for small maps
    tf(4,1:3) = -enu_trans;
    mapout.pc = pctransform(mapin.pc,affine3d(tf));
    mapout.gps_origin = gps1;
end