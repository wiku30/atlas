function [map,err] = frame_init(pc,trace_gps,trace_re,varargin)
    trace_enu = get_pos(trace_gps);
    map.gps_origin = trace_gps(1,2:4);
    [tf,err] = sync(trace_re,trace_enu);
    map.pc = pctransform(pc,tf);
    if nargin>=4
        a=cell2mat(varargin{1})
        figure(a);
        hold on;
        ttrace(trace_enu);
        ttrace(pctransform(pointCloud(trace_re(:,2:4)),tf).Location);
        hold off;
    end
end