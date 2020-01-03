function M = get_pos(in,varargin)
    n=size(in,1);
    if nargin==1
        x0=in(1,2);
        y0=in(1,3);
        z0=in(1,4);
    else
        rf = varargin{1};
        x0=rf(1,2);
        y0=rf(1,3);
        z0=rf(1,4);
    end
    for i=1:n
        M(i,1)=in(i,1);
        [M(i,2),M(i,3),M(i,4)] = geodetic2enu(in(i,2),in(i,3),in(i,4),x0,y0,z0,wgs84Ellipsoid);
    end
end