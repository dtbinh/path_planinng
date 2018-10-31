%% Description 
% this code simulates a scenario. 

%% Map setting
map_dim = 20;
lx = 10; ly = 10; % size of map in real coordinate 
res = lx / map_dim;
custom_map=makemap(20); % draw obstacle interactively

%% Generate map occupancy grid object 

map = robotics.OccupancyGrid(flipud(custom_map),1/res);
show(map);
[target_xs,target_ys,tracker]=set_target_tracker; % assign target and tracker
%% Get score map from each target point

vis_cost_set = []; % row : time / col : angle index 
N_azim = 10;
DT_set = [];
for t = 1:length(target_xs) % -1 is due to some mistake in set_target_tracker function 
    target_position = [target_xs(t), target_ys(t) 0]';
    ray_len = norm(target_position(1:2) - tracker);
    ray_lens = ray_len * ones(N_azim,1);
    angles = linspace(0,2*pi,N_azim+1);
    angles(end) = [];
    
    cast_res = zeros(1,N_azim); % 1 for hit and 0 for free rays    t
    collisionPts=map.rayIntersection(target_position,angles,ray_len); % the NaN elements are not hit, 
    collisionIdx=find(~isnan(collisionPts(:,1))); % collided index 
    cast_res(collisionIdx) = 1;
    DT = signed_distance_transform([cast_res cast_res cast_res]); % periodic distance transform 
    DT = DT(N_azim+1:2*N_azim); % extracting 
    DT_set = [DT_set ; DT ];
    vis_cost = max(DT) - DT + 1; 
    vis_cost_set(t,:) = vis_cost;    
end
%% Feasible region convex division for LP problem 

% for now, the feasible region for solving discrete path is just rectangle
% (TODO: extension for general affine region)
H = length(target_xs);
x_range = 12;
y_range = 6;
feasible_domain_x = [tracker(1) - x_range/2 tracker(1) + x_range/2];
xl = feasible_domain_x(1);
xu = feasible_domain_x(2);
feasible_domain_y = [tracker(2) - y_range/2  tracker(2)  + y_range/2];
yl = feasible_domain_y(1);
yu = feasible_domain_y(2);

% plot the problem 
show(map)
hold on 
plot(target_xs,target_ys,'r*')
plot(tracker(1),tracker(2),'ko')
patch([xl xu xu xl],[yl yl yu yu],'red','FaceAlpha',0.1)

% A_sub, b_sub : inequality matrix of each sub division region (sigma Nh  pair)
Nh = zeros(1,H); % we also invesigate number of available regions per each time step 
angles = linspace(0,2*pi,N_azim+1);
S = [];
A_sub  = {};
b_sub = {};
N_vis_show = 3; % shows N largest vis score 
d_ref = norm([target_xs(1) target_ys(1)]' - tracker);

for h = 1:H
    Nk = 0; % initialize number of valid region
        
    for k = 1:N_azim    
        % Bounding lines of the k th pizza segment !
        theta1 = 2*pi/N_azim * (k-1);
        theta2 = 2*pi/N_azim * (k);        
        v1 = [cos(theta1), sin(theta1)]'; 
        v2 = [cos(theta2) , sin(theta2)]'; 
        % If this holds, then one of the two line is in the box 
        if ( is_in_box(v1,[target_xs(h) ; target_ys(h)],[xl xu],[yl yu]) || is_in_box(v2,[target_xs(h) ; target_ys(h)],[xl xu],[yl yu]) )
            if(DT_set(h,k)) % occlusion and collision rejection
                [A,b] = get_ineq_matrix([target_xs(h) ; target_ys(h)],v1,v2);        
                Nk = Nk +1;
                A_sub{h}{Nk} = A; b_sub{h}{Nk}=b; % inequality constraint
                S=[S vis_cost_set(h,k)]; % visbiility cost of the region 
            end
        end          
    end
    
    [sorted_val, indices] = sort(vis_cost_set(h,:));
    goods = indices(1:N_vis_show);
   
    for i = 1:N_vis_show
        good = goods(i);
        draw_circle_sector([target_xs(h) target_ys(h)],2*pi/N_azim * (good-1),2*pi/N_azim * (good),d_ref,'g',1/sorted_val(i)*0.3);        
    end
    
    Nh(h) = Nk; % save the available number of region
end

%% Generation of optimal sequence (refer lab note)


%%%
% Optimization variables 
% X = [x1 y1 x2 y2 ... xH yH  ||  d1x d1y ... dHx dHy || j1x j1y ... jHx jHy || z_1,1 z_1,2 ... z_1,N1 | ..| z_H,1 ... z_H,N_H]
% w_v: weight for visibility 
%%%


%%%
% Parameters
% w_j : weight for jerk 
% w_v: weight for visibility 
%%%
H  = length(target_xs) ; % total horizon 
N_var = 2*H + 2*H + 2*(H-2) + length(S); % x,y,dx,dy,jx,jy,z, X+, X- , Y+, Y-, D 
w_j = 1;
w_v = 100;


% objective function : sum of travel (1 norm), sum of jerk, sum of visibility cost  
f = [zeros(1,2*H),  ones(1,2*H) ,  w_j * ones(1,2*(H-2)), w_v*S];

% equality constraint
Aeq = zeros(H,N_var);
insert_idx = 2*H + 2*H  + 2*(H-2) + 1;
    
for h = 1:H
    Aeq(h,insert_idx:insert_idx+Nh(h)-1) = ones(1,Nh(h));
    insert_idx = insert_idx+Nh(h)  ;
end
beq = ones(H,1);
   

% inequality 1 : travel distance auxiliary variables
Aineq1 = zeros(4*H,N_var); bineq1 = zeros(4*H,1);
insert_mat_xy = [-1 0 ; 1 0 ; 0 -1 ; 0 1];
insert_mat_d = [-1 0 ; -1 0; 0 -1; 0 -1];
insert_row = 1;
insert_col = 1;


for h = 1:H
    if h == 1
        Aineq1(insert_row:insert_row+3,insert_col:insert_col + 1) = insert_mat_xy; bineq1(insert_row:insert_row+3) = [-tracker(1) tracker(1) -tracker(2) tracker(2)]';
        Aineq1(insert_row:insert_row+3,insert_col + 2*H  : insert_col + 2*H +1 ) = insert_mat_d;
    else
        Aineq1(insert_row:insert_row+3,insert_col-2:insert_col+1) = [-insert_mat_xy insert_mat_xy];
        Aineq1(insert_row:insert_row+3,insert_col + 2*H : insert_col + 2*H + 1) = insert_mat_d;        
    end    
    insert_row = insert_row + 4;
    insert_col = insert_col + 2;
end



% inequality 2 : jerk auxiliary variables

insert_row = 1;
insert_col = 1;
Aineq2 = zeros(4*(H-2),N_var);
bineq2 = zeros(4*(H-2),1);
insert_mat_xy = [-1 0 ; 1 0 ; 0 -1 ; 0 1];
insert_mat_xy = [-insert_mat_xy 3*insert_mat_xy -3*insert_mat_xy insert_mat_xy];

for h = 1:H - 2
    if h == 1
        Aineq2(insert_row:insert_row+3,insert_col:insert_col + 5) = insert_mat_xy(:,3:end); bineq2(insert_row:insert_row+3) = [-tracker(1) tracker(1) -tracker(2) tracker(2)]';
        Aineq2(insert_row:insert_row+3,insert_col + 4*H   : insert_col + 4*H +1  ) = insert_mat_d;
    else
        Aineq2(insert_row:insert_row+3,insert_col-2:insert_col+5) = insert_mat_xy;
        Aineq2(insert_row:insert_row+3,insert_col + 4*H  : insert_col + 4*H + 1 ) = insert_mat_d;        
    end    
    insert_row = insert_row + 4;
    insert_col = insert_col + 2;
end


% inequality 3 : sub division inequality (sum of Nh (h= 1... H) many constraints)
M = 1e+2; % some large number (big enough but not too be big)

dim = 2; % currently 2D
Aineq3 = zeros(dim * sum(Nh),N_var);
bineq3 = M*ones(dim * sum(Nh),1);

insert_blk_row = 1;
for h = 1:H 
    for k = 1: Nh(h)
        Aineq3(2*(insert_blk_row-1)+1:2*(insert_blk_row),2*(h-1)+1:2*(h)) = A_sub{h}{k};
        Aineq3(2*(insert_blk_row-1)+1:2*(insert_blk_row), 6*H-4 + (insert_blk_row)) = -b_sub{h}{k} + M; 
        insert_blk_row = insert_blk_row + 1;
    end   
end

% inequality 4: interval distance limit (1 norm) 
d_max = 3;
Aineq4 = zeros(H,N_var); bineq4 = d_max * ones(H,1);
for h = 1:H
    Aineq4(h,2*H + 2*(h-1) + 1 : 2*H + 2*h) = ones(1,2);     
end


intcon = 6*H -4 + 1 : N_var;
lb = -inf * ones(N_var,1); lb(intcon) = 0; lb(1:2:2*H) = xl;  lb(2:2:2*H) = yl;
ub = inf * ones(N_var,1); ub(intcon) = 1; ub(1:2:2*H) = xu; ub(2:2:2*H) = yu;
tic
sol = intlinprog(f,intcon,[Aineq1 ; Aineq2;  Aineq3 ; Aineq4 ],[bineq1; bineq2; bineq3; bineq4],Aeq,beq,lb,ub);
toc

%% Anaylsis

% variables parsing 
waypoints_x = sol(1:2:2*H);
waypoints_y = sol(2:2:2*H);
plot(waypoints_x,waypoints_y,'ks-','LineWidth',1)
plot([tracker(1) waypoints_x(1)],[tracker(2) waypoints_y(1)],'ks-','LineWidth',1);

select_region_seq = zeros(1,H);
inspect_idx = 6*H -4 + 1;

% for h = 1:H
%     inspect = sol(inspect_idx : inspect_idx + Nh(h) -1);    
%     select_region_seq(h) = find(uint8(inspect)==1);
%     inspect_idx = inspect_idx + Nh(h);   
% end










