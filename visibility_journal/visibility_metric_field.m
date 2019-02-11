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
% custom_map=makemap(map_dim); % draw obstacle interactively
% or load the simple example 
%load('prob_setting.mat')
% get boundary
% sdf = signed_distance_transform(custom_map);
% boundary_map = (sdf == 0);

%% Generate map and cluster the obstacles 
figure
% map = robotics.OccupancyGrid(flipud(boundary_map),1/res);
% load('map.mat')
h =show(map);
h.AlphaData = 0.2;
occ_mat = map.occupancyMatrix;
occ_mat(occ_mat>0.5) = 1; occ_mat(occ_mat<0.5) = 0; 
dist_map=signed_distance_transform(occ_mat);

[r_edge,c_edge] = find(dist_map <= 0); % filled data
% [r_edge,c_edge] = find(dist_map == 0); % hull data
sample_pnts = map.grid2world([r_edge c_edge]);
hold on
% plot(sample_pnts(:,1),sample_pnts(:,2),'ko','MarkerSize',1)

% cluster 
eps = res*1.5;
N_min = 4;
cluster_idx =DBSCAN(sample_pnts,eps,N_min);
rects = {}; T_rects = {};
% box fitting 
for i = 1:max(cluster_idx)
    [rect_pnts,T]=box_fit(sample_pnts(find(cluster_idx == i),:)); 
    rects{i} = rect_pnts;
    T_rects{i} = T;
end

% target and fitted box 
figure
axis([map.XWorldLimits map.YWorldLimits])
hold on 
% draw fitting box 
for i = 1:max(cluster_idx)
    h_rect = patch(rects{i}(1,:),rects{i}(2,:),'m','EdgeColor','c','FaceColor','w','LineWidth',4);
end

target = [5 4] + [0.5 0];
mu = target;
theta_dist = 0; % covariance angle 
R = rot2(theta_dist); % linear transformation 
sigma1 = sqrt(0.05);
sigma2 = sqrt(0.05);

Sigma = R*diag([sigma1^2; sigma2^2])*R';
dist_window = [4 4];
x1 = linspace (target(1)-dist_window(1),target(1)+dist_window(1),200);
x2 = linspace (target(2)-dist_window(2),target(2)+dist_window(2),200);
[X1,X2] = meshgrid(x1,x2);
F = mvnpdf([X1(:) X2(:)],mu,Sigma);
F = reshape(F,length(x2),length(x1));    
[~,h_pdf]=contour(x1,x2,F,[ .01 .05:.1:.95 .99 .999 .9999]);
% surfc(x1,x2,F);

% let's find the iso-contours 
% first generate the points on a circle of radius chi2(alpha)
T_elip = SE2(R);
T_elip.t = mu;
alpha = 0.05;  % 100(1-alpha) percentile 
chi_val = -2*log(alpha);
Nb = 20; % pnts for plot 
th = linspace(0,2*pi,Nb);

% inscrbing box 
r1 = sqrt(chi_val)*sigma1;
r2 = sqrt(chi_val)*sigma2;
integration_window_pnts = T_elip*[-r1 r1 r1 -r1;-r2 -r2 r2 r2];
% h_HDR_approx=patch(integration_window_pnts(1,:),integration_window_pnts(2,:),'m','EdgeColor','r','FaceColor','none','LineWidth',2);

pnts_on_circle = sqrt(chi_val)*[cos(th) ; sin(th)];
pnts_on_elip = R*diag([sigma1 sigma2])*pnts_on_circle+mu'; 
h_HDR = plot(pnts_on_elip(1,:),pnts_on_elip(2,:),'r-','LineWidth',2);

% legend([hh hhh hhhh hhhhh h_rect],{'pcl','centroid','Major','Minor','fit box'});

legend([h_rect h_pdf  h_HDR ],{'box fit','target pdf','95% HDR'})

% surf(x1,x2,F,'EdgeColor','none','FaceAlpha',0.5)
% caxis([min(F(:)),max(F(:))]);
colormap(jet)
axis equal

%% let's generate field 
score_map = zeros(map.GridSize);
sensor_model.range = 10;
sensor_model.FOV = pi/10;
for r = 1:map.GridSize(1)
    for c = 1:map.GridSize(2)
        loc = map.grid2world([r c]);
        if ~is_within_box(rects,T_rects,loc,1e-1)
           score_map(r,c) = eval_visibility(map.XWorldLimits,map.YWorldLimits,rects,sensor_model,0.2,mu,Sigma,loc);
        end
    end
end

%% field plot
figure
[Ys,Xs] = meshgrid(linspace(map.XWorldLimits(1),map.XWorldLimits(2),map.GridSize(1)),linspace(map.YWorldLimits(1),map.YWorldLimits(2),map.GridSize(2)));
surf(Ys,10-Xs,score_map,'EdgeColor','none','FaceAlpha',0.4)
colormap(jet)
shading interp
grid off
