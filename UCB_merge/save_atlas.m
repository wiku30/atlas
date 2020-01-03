function a=save_atlas(t)
    n=size(t,2)
    for k=1:n
        k
        [aa{k}(:,1),aa{k}(:,2)]=i2xy(cell2mat(t{k}.keys));
        a=double(aa{k});
        format long;
        save(['map_', num2str(k), '.tile'], 'a','-ascii');
        a=aa;
    end
end