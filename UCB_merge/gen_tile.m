function table = gen_tile(maps)
prec = 5;
    for k=1:size(maps,2)
        k
        co = pcdownsample(maps(k).pc,'gridAverage',2.5).Location;
        n = size(co,1);
        table{k} = containers.Map('KeyType','int32','ValueType','int8');
        for i=1:n
            x=int32(round(co(i,1)/prec));
            y=int32(round(co(i,2)/prec));
            ins(table{k},x,y,1);
        end
    end
end

function ins(m,x,y,v)
    m(xy2i(x,y))=v;
end

function v=val(m,x,y)
    v=m(xy2i(x,y));
end