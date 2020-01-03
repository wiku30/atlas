clear;
load('co.mat')
pc1 = pcread('1-.pcd');
pc4 = pcread('4-.pcd');
map1 = frame_init(pc1,gpsf1,re1);
map4 = frame_init(pc4,gpsf4,re4);

map = merge_map(map1,map4);

showmap(merge_map(map1,map4,-1),3);
showmap(map,4);




