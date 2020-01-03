function showmap(varargin)
    if(nargin>=2)
        figure(varargin{2});
    end
    pcshow(varargin{1}.pc);
end