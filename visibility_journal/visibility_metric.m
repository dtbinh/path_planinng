%% Description 
% this code generates visiblity field for a target represented by a
% gaussian model 
addpath('C:\Users\junbs\Documents\path_planinng\ASAP1','C:\Users\junbs\Documents\path_planinng\multi_target_tracking\','..\plotregion\');
addpath('C:\Users\junbs\Documents\path_planinng\multi_target_tracking\polytopes_2017_10_04_v1.9')
%% Map setting

map_dim = 40;
world_lx = 10; world_ly = 10; % size of map in real coordinate 
res = world_lx / map_dim;
custom_map=makemap(map_dim); % draw obstacle interactively
% or load the simple example 
%load('prob_setting.mat')
% get boundary
sdf = signed_distance_transform(custom_map);
boundary_map = (sdf == 0);

%% Generate map and cluster the obstacles 
figure
map = robotics.OccupancyGrid(flipud(boundary_map),1/res);
h =show(map);
h.AlphaData = 0.2;
occ_mat = map.occupancyMatrix;
occ_mat(occ_mat>0.5) = 1; occ_mat(occ_mat<0.5) = 0; 
dist_map=signed_distance_transform(occ_mat);

% [r_edge,c_edge] = find(dist_map <= 0); % filled data
[r_edge,c_edge] = find(dist_map == 0); % hull data
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

%% target and fitted box 
figure
axis([map.XWorldLimits map.YWorldLimits])
hold on 
% draw fitting box 
for i = 1:max(cluster_idx)
    h_rect = patch(rects{i}(1,:),rects{i}(2,:),'m','EdgeColor','c','FaceColor','w','LineWidth',4);
end

target = [5.5 4];
mu = target;
theta_dist = pi/6; % covariance angle 
R = rot2(theta_dist); % linear transformation 
sigma1 = sqrt(0.2);
sigma2 = sqrt(0.1);

Sigma = R*diag([sigma1^2; sigma2^2])*R';
dist_window = [4 4];
x1 = linspace (target(1)-dist_window(1),target(1)+dist_window(1),200);
x2 = linspace (target(2)-dist_window(2),target(2)+dist_window(2),200);
[X1,X2] = meshgrid(x1,x2);
F = mvnpdf([X1(:) X2(:)],mu,Sigma);
F = reshape(F,length(x2),length(x1));    
[~,h_pdf]=contour(x1,x2,F,[ .01 .05:.1:.95 .99 .999 .9999]);
% surfc(x1,x2,F);
legend([h_rect h_pdf],{'box fit','target pdf'})

% surf(x1,x2,F,'EdgeColor','none','FaceAlpha',0.5)
% caxis([min(F(:)),max(F(:))]);
colormap(jet)
axis equal

%% observer and shading area 
observer = [4.5 1.5];
h_target = plot(observer(1),observer(2),'k^','MarkerFaceColor','k','MarkerSize',10);
legend([h_rect h_pdf h_target],{'box fit','target pdf','observer'})

% bearing vector plot 
h_bearing=quiver(observer(1),observer(2),mu(1)-observer(1),mu(2)-observer(2),'k--','LineWidth',2,'MarkerFaceColor','k');

% occlusion culling 
% boundary : need to be set 
%plotregion(-A,-b,[xl yl]',[xu yu]',[0 0.5 0],0.4)

% shading region 
As = {};
bs = {};
for n = 1:length(rects)
    [A,b]=get_shading(observer,rects{n});
    A=[A;eye(2) ; -eye(2)]; b = [b ; map.XWorldLimits(2) ; map.YWorldLimits(2) ; map.XWorldLimits(1) ;map.YWorldLimits(1)];     
    As{n} = A;
    bs{n} = b;    
    axis([map.XWorldLimits map.YWorldLimits])
    vertices=lcon2vert(A,b);
    hull_idx=convhull(vertices);
    h_shade = patch(vertices(hull_idx,1),vertices(hull_idx,2),'k','FaceAlpha',0.2,'EdgeColor','none');
end
for i = 1:max(cluster_idx)
    h_rect = patch(rects{i}(1,:),rects{i}(2,:),'m','EdgeColor','c','FaceColor','w','LineWidth',4);
end

legend([h_rect h_pdf h_target],{'box fit','target pdf','observer'})

%% Integration window 
% first, we find the level set c = 1/e of peak
% e = 2.718;
% peak_val = 1/(2*pi*sigma1*sigma2);
% c = 1/e*peak_val;
% r1 = sqrt(2*sigma1^2*log(1/(2*pi*c*sigma1*sigma2)));
% r2 = sqrt(2*sigma2^2*log(1/(2*pi*c*sigma1*sigma2)));

% integration_window_pnts = T_elip*[-r1 r1 r1 -r1;-r2 -r2 r2 r2];

% patch(integration_window_pnts(1,:),integration_window_pnts(2,:),'m','EdgeColor','r','FaceColor','none','LineWidth',2)


% let's find the iso-contours 
% first generate the points on a circle of radius chi2(alpha)
T_elip = SE2(R);
T_elip.t = mu;
alpha = 0.05;  % 100(1-alpha) percentile 
chi_val = -2*log(alpha);
Nb = 20; % pnts for plot 
th = linspace(0,2*pi,Nb);
pnts_on_circle = sqrt(chi_val)*[cos(th) ; sin(th)];
pnts_on_elip = R*diag([sigma1 sigma2])*pnts_on_circle+mu'; 
h_HDR = plot(pnts_on_elip(1,:),pnts_on_elip(2,:),'r-','LineWidth',2);

% legend([hh hhh hhhh hhhhh h_rect],{'pcl','centroid','Major','Minor','fit box'});

fun_xy = @(x1,x2) reshape(mvnpdf([x1(:) x2(:)],mu,Sigma),size(x1)); % original function
% fun_xy_trans = @(x1_,x2_) fun_xy(sigma1*dot(R(1,:),[x1_ x2_])+mu(1),sigma2*dot(R(2,:),[x1_ x2_])+mu(2))*sigma1*sigma2; % linear transform 
fun_xy_trans = @(x1_,x2_) fun_xy(sigma1*(R(1,1).*x1_+R(1,2).*x2_)+mu(1),sigma2*(R(2,1)*x1_ + R(2,2)*x2_)+mu(2))*sigma1*sigma2; % linear transform 
fun_polar = @(r,theta) fun_xy_trans(r.*cos(theta),r.*sin(theta)).*r;
score = integral2(fun_polar,0,sqrt(chi_val),0,2*pi);   

legend([h_rect h_pdf h_target h_HDR],{'box fit','target pdf','observer','95% HDR'})




