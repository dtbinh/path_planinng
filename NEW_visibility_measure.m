%% Description 
% this code simulates a scenario. 
addpath('C:\Users\junbs\Documents\path_planinng\ASAP1','C:\Users\junbs\Documents\path_planinng\multi_target_tracking\');


%% Map setting
map_dim = 20;
world_lx = 10; world_ly = 10; % size of map in real coordinate 
res = world_lx / map_dim;
custom_map=makemap(20); % draw obstacle interactively
% or load the simple example 
%load('prob_setting.mat')
%% Generate map occupancy grid object 
map = robotics.OccupancyGrid(flipud(custom_map),1/res);
show(map);
[target_xs,target_ys,tracker]=set_target_tracker; % assign target and tracker

%% score map 
lx = 4; ly = 4; 

show(map);
title('')
xlabel('')
ylabel('')
axis([0 10 0 10 ])
draw_t_step = 3;
xc = [target_xs(draw_t_step) target_ys(draw_t_step)];
draw_cost_vec=vis_cost_set(draw_t_step,:);
max_cost = max(draw_cost_vec);
min_cost = min(draw_cost_vec);
hold on 
% h=plot(target_xs,target_ys,'r^-','LineWidth',2);
% h.Color(4) = 0.3;
plot(target_xs(draw_t_step),target_ys(draw_t_step),'r^','LineWidth',5);
Nx = 10;
Ny = 10;


dom_x = linspace(max(target_xs(draw_t_step)-lx,0), min(target_xs(draw_t_step)+lx,world_lx),Nx);
dom_y = linspace(max(target_xs(draw_t_step)-ly,0), min(target_xs(draw_t_step)+ly,world_ly),Ny);
dx = 2*lx/(Nx-1);
dy = 2*lx/(Ny-1);
ray_len = 5;
N_ray = 10;
FOV = 2*pi/3;
dom_x = 6;
dom_y = 9;
S_mesh = zeros(Nx,Ny);
x_idx = 1;
y_idx = 1;
for x = dom_x
    for y = dom_y                 
        plot(x,y,'ks')
        bearing_ang = atan2(xc(2)-y,xc(1)-x);
        th_min = bearing_ang - FOV/2;
        th_max = bearing_ang + FOV/2;
        th_set = linspace(th_min,th_max,N_ray);
        th_d = th_set(2)-th_set(1);
        
        ls = [];
        end_pnts = [];
        for th = th_set
            p1 = [x,y];            
            p2 = [x,y] + ray_len * [cos(th), sin(th)];   
            end_pnt=map.rayIntersection([p1 0],th,ray_len);
            if isnan(end_pnt)
                plot([p1(1) p2(1)],[p1(2) p2(2)],'b-','LineWidth',2)
%                 plot([p2(1) p2(2)],'ko')
                ls = [ls ray_len];
                end_pnts = [end_pnts ; p2];
            else
                plot([p1(1) end_pnt(1)],[p1(2) end_pnt(2)],'b-','LineWidth',2)
%                 plot(end_pnt(1),end_pnt(2),'ko');
                ls = [ls norm(end_pnt - p1)];
                
                end_pnts = [end_pnts ; end_pnt];
            end                        
        end
        
        S_vis = 0;
        for i = 1:N_ray-1
            plot([end_pnts(i,1) end_pnts(i+1,1)],[end_pnts(i,2) end_pnts(i+1,2)],'b-','LineWidth',2)
            S_vis = S_vis + ls(i)*ls(i+1)*th_d;
        end

        S_mesh(x_idx,y_idx) = S_vis;
        y_idx = y_idx + 1;
    end
    x_idx = x_idx +1;
end

% max_S = max(max(S_mesh));
% min_S = min(min(S_mesh));
% color_scale = jet(100);
% for x_idx = 1:Nx    
%     for y_idx = 1:Ny
%         coll = color_scale(max(floor(100*(max_S - S_mesh(x_idx,y_idx)) /(max_S-min_S)),1),:);
%         r_x = dom_x(x_idx); dx = dom_x(2)-dom_x(1);
%         r_y = dom_y(y_idx); dy = dom_y(2)-dom_y(1);                        
%         patch([r_x-dx/2 r_x+dx/2 r_x+dx/2 r_x-dx/2],[r_y-dy/2 r_y-dy/2 r_y+dy/2 r_y+dy/2],coll);      
%     end    
% end


%%


