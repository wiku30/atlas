function ttrace(g)
    if size(g,2) == 4
        plot3(g(:,2),g(:,3),g(:,4),'.');
    else
        plot3(g(:,1),g(:,2),g(:,3),'.');
    end
    axis equal;
end