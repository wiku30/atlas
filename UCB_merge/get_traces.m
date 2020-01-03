clear


re1 = load('dump1-.txt');
re4 = load('dump4-.txt');
gps1 = load('g1_parsed.txt');
gps4 = load('g4_parsed.txt');

lim1 = get_lim(re1);
lim4 = get_lim(re4);

con = 4000000000;
con = 0;


gpsf1=(time_filter(gps1,lim1+con));
gpsf4=(time_filter(gps4,lim4+con));

enu1 = get_pos(gpsf1);
enu4 = get_pos(gpsf4,gpsf1);

save('co.mat','re1','re4','enu1','enu4','gpsf1','gpsf4');

figure(10);
ttrace(re1);
figure(11);
ttrace(enu1);
figure(40);
ttrace(re4);
figure(41);
ttrace(enu4);

%save('coordinates.mat',gps)

function lim=get_lim(in)
    n=size(in,1);
    lim(1)=in(1,1)*1e+6
    lim(2)=in(n,1)*1e+6
end

function b = time_process(a,c)
    b=a;
    b(:,1)=(a(:,1))*1e-9 -c(1:1)*1e-3;
end

function utm_set = time_filter(utm,lim)
    ind = find(ifbetween(utm(:,1),lim(1),lim(2)));
    utm_set = utm(ind,:);
end

function res = ifbetween (a,l,r)
    res = ((a-l).*(a-r)<0);
end