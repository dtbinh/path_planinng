%% Description 
% this code generates visiblity field for a target represented by a
% gaussian model 
addpath('..\ASAP1','..\multi_target_tracking\','..\plotregion\');
addpath('..\multi_target_tracking\polytopes_2017_10_04_v1.9')
addpath(genpath('..\robot10.1'))
addpath(genpath('.\Matlab_Polygons_intersection\'))
addpath(genpath(pwd))

%% Map setting

map_dim = 40;
world_lx = 10; world_ly = 10; % size of map in real coordinate 
res = world_lx / map_dim;
custom_map=makemap(map_dim); % draw obstacle interactively
% or load the simple example 
%load('prob_setting.mat')
% get boundary
% sdf = signed_distance_transform(custom_map);
% boundary_map = (sdf == 0);

%% Generate map 

map = robotics.OccupancyGrid(flipud(custom_map),1/res);
target = [5 4];
vel = [1 0.5]; % predicted velocity 



% load('map.mat')
% h =show(map);
hold on
% h.AlphaData = 0.2;
occ_mat_origin = map.occupancyMatrix;
occ_mat_origin(occ_mat_origin>0.5) = 1; occ_mat_origin(occ_mat_origin<0.5) = 0; 
% trace 
N_trace = 3;
occ_mat = zeros(size(occ_mat_origin));
for trans_val = 0:N_trace
    occ_mat_trans=shiftmatrix(occ_mat_origin,1,[-floor(vel(2)/res/N_trace*trans_val) floor(vel(1)/res/N_trace*trans_val)],0);
    occ_mat = double(logical(occ_mat) | logical(occ_mat_trans));
end
dist_map=signed_distance_transform(occ_mat);
dist_map_origin = signed_distance_transform(occ_mat_origin);
boundary_map = dist_map == 0;
dist_map(dist_map<0)=0;
dist_map = 0.1*dist_map;

[Ys,Xs] = meshgrid(linspace(map.XWorldLimits(1),map.XWorldLimits(2),map.GridSize(1)),linspace(map.YWorldLimits(1),map.YWorldLimits(2),map.GridSize(2)));
surf(Ys,10-Xs,dist_map,'EdgeColor','none','FaceAlpha',0.8)
colormap(pink)
% shading interp;
colorbar



% hold on 
% h_target = plot3(target(1),target(2),100,'m^','MarkerFaceColor','b','MarkerSize',20);
title('EDT')
axis equal

%% generate visibility field

score_map = zeros(map.GridSize);
for r = 1:map.GridSize(1)
    for c = 1:map.GridSize(2)
        loc = map.grid2world([r c]);
        
        [end_pnts,mid_pnts]=map.raycast(target,loc);

        dist_val_along_ray = zeros(1,length(mid_pnts));        
        for i = 1:length(mid_pnts)
            dist_val_along_ray(i) = dist_map(mid_pnts(i,1),mid_pnts(i,2));
        end
        visi_val =min(dist_val_along_ray);        
        score_map(r,c) = visi_val;               
    end
end
figure
surf(Ys,10-Xs,score_map,'EdgeColor','none','FaceAlpha',0.4)
hold on
colormap(jet)
shading interp
h_target = plot3(target(1),target(2),100,'m^','MarkerFaceColor','b','MarkerSize',20);
grid off
colorbar

% draw original map 
[occ_idx_r,occ_idx_c] = find(map.occupancyMatrix >0.5); 
for idx =1: length(occ_idx_r)
    loc=map.grid2world([occ_idx_r(idx) occ_idx_c(idx)]);
    dx = res/2;
    fill3([loc(1)-dx loc(1)+dx loc(1)+dx loc(1)-dx],[loc(2)-dx loc(2)-dx loc(2)+dx loc(2)+dx],[100 100 100 100],'k')
    
end
h_vel = quiver(target(1),target(2),2*vel(1),2*vel(2),'m','LineWidth',4,'MaxHeadSize',5);
legend([h_target h_vel],{'target','velocity'})
title('visibility field')





