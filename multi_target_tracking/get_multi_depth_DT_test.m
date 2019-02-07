%% load data 
% loading example data
addpath('../STOMP/DBSCAN/')
load('map2/map2_solution.mat')
ray_origin = [4,1,1];

%% Phase 1 : investigate the hit distance toward every direction 
    
    N_azim = length(azim_set);
    N_elev = length(elev_set);
    hold on 
    hit_dist = zeros(N_azim,N_elev);
    show(map3)
    rays = [];
    
    ray_ends = {};
    free_margin = 5;
   hit_dist_inflated = hit_dist;
    for azim_idx = 1:N_azim
        for elev_idx = 1:N_elev
            hit_dist(azim_idx,elev_idx) = raycast3D(map3,ray_origin,azim_set(azim_idx),elev_set(elev_idx),max_ray,ray_cast_res);     % the hit distance into that direction        
            if hit_dist(azim_idx,elev_idx) == max_ray
                hit_dist(azim_idx,elev_idx) + free_margin; % this is for clear 
            else
            end
            azim = azim_set(azim_idx);
            elev = elev_set(elev_idx);
            ray_ends{azim_idx,elev_idx} = ray_origin + hit_dist(azim_idx,elev_idx)*[cos(elev)*cos(azim) cos(elev)*sin(azim) sin(elev)];
            rays = [rays ; hit_dist(azim_idx,elev_idx)*[cos(elev)*cos(azim) cos(elev)*sin(azim) sin(elev)]];
            plot3([ray_origin(1) ray_ends{azim_idx,elev_idx}(1)],[ray_origin(2) ray_ends{azim_idx,elev_idx}(2)],[ray_origin(3) ray_ends{azim_idx,elev_idx}(3)],'k-','LineWidth',1);
        end
    end

    %% Phase 2: DBSCAN with defined metric 
    N_pnts = 3; epsilon = 1;
    DBSCAN(rays,)
    
    
    
    
