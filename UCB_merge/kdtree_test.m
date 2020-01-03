if 0
    for i=1:5
        m(i)=load_map(['A/maps/A',num2str(i)]);
    end
    

end
clear();
N=5;
prec=5.0;
load('ATmaps.mat','AA');
m=AA;
ori = m(3).gps_origin;
for i=1:5
    m(i)=change_origin(m(i),ori);
end
for k=1:5
    k
    co = pcdownsample(m(k).pc,'gridAverage',2.5).Location;
    n = size(co,1);
    table{k} = containers.Map('KeyType','int32','ValueType','int8');
    for i=1:n
        x=int32(round(co(i,1)/prec));
        y=int32(round(co(i,2)/prec));
        ins(table{k},x,y,1);
    end
end

function ins(m,x,y,v)
    m(xy2i(x,y))=v;
end

function v=val(m,x,y)
    v=m(xy2i(x,y));
end




