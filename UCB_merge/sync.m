function [tf_best,err] = sync(re1,enu1)

    init_tf = eye(4);
    init_tf(4,1:3) = re1(1,2:4);

    pc_re = co2pc(re1);
    pc_enu = co2pc(enu1) ;

    n=12;

    for i=1:n
        w = 3.1415926 * 2 * i / n;
        init_tf(1:2,1:2) = [cos(w),sin(w);-sin(w),cos(w)];
        [tf{i},pc_sy{i},rmse(i)] = pcregistericp(pc_re,pc_enu,'InitialTransform',affine3d(init_tf));
    end

    err = 1e+243;

    tf_best=affine3d(eye(4));

    for i=1:n
        if rmse(i)<err
            err=rmse(i);
            tf_best=tf{i};
        end
    end
    if err>10
        warning('Error too large! Are they matched?');
    end
end

function pc = co2pc(co)
    pc = pointCloud(co(:,2:4));
end