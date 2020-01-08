zone={}
for k=1:6
    a=cell2mat(table{k}.keys);
    for i=1:size(a,2)
        [zone{k}(i,1),zone{k}(i,2)]=i2xy(a(i));
    end
    zone{k} = zone{k} * 5;
end

figure(1);
hold on
    scatter(zone{1}(:,1),zone{1}(:,2), 1,'.');
    scatter(zone{2}(:,1),zone{2}(:,2), 1, 'x');
    scatter(zone{3}(:,1),zone{3}(:,2), 1, 'o');
    scatter(zone{4}(:,1),zone{4}(:,2), 1, '*');
    scatter(zone{5}(:,1),zone{5}(:,2), 1, '+');
    scatter(zone{6}(:,1),zone{6}(:,2), 1);
    
    axis equal;
