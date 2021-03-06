function [DT_set,pinch_bin] = get_multi_depth_DT(map3,ray_origin,azim_set,elev_set,max_ray,min_ray,ray_cast_res,clustering_delta_r,raycast_plot)
    % This function get a multi-layered distance field 
    % clustering epsilon : clustering_delta_r 
    % ray cast res : ray cast resolution (normally 1/map3.res)
    
    %% Phase 1 : investigate the hit distance toward every direction 
    N_azim = length(azim_set);
    N_elev = length(elev_set);
    
    hit_dist = zeros(N_azim,N_elev);
    
    for azim_idx = 1:N_azim
        for elev_idx = 1:N_elev
            hit_dist(azim_idx,elev_idx) = raycast3D(map3,ray_origin,azim_set(azim_idx),elev_set(elev_idx),max_ray,ray_cast_res);    % the hit distance into that direction        
        end
    end
    
    % do you need plot (in new figure) ?
    if raycast_plot
        fig_h=plot_raycast(map3,ray_origin,hit_dist,azim_set,elev_set,'k-',0.2,1)
%         plot_raycast(map3,ray_origin,hit_dist(:,1),azim_set,elev_set(1),'k-',0.2,1) % 2D version 
    end
    
    
   %% Phase 2: clustering the hit distance 
   hit_dist_flat=reshape(hit_dist,1,[]); % histogram

%    hit_dist_flat=hit_dist(:,1)'; % for illustration
    
   % tuning!! 
   N_min_cluster = 4; 
   free_margin = 10; % for clear clustering for the free region 
   hit_dist_flat_for_clustering=hit_dist_flat; 
   hit_dist_flat_for_clustering (hit_dist_flat == max_ray ) = hit_dist_flat_for_clustering (hit_dist_flat == max_ray ) + free_margin; 
   IDX=DBSCAN(hit_dist_flat_for_clustering',clustering_delta_r,N_min_cluster); % cluster delta r 
   IDX_back = reshape(IDX,size(hit_dist)); 
   if raycast_plot
      figure
      hold on
      title('hit distance')
      stem(azim_set,hit_dist(:,1),'k-');
      plot([0 azim_set(end)],[max_ray max_ray],'g--')
      xlabel('azimuth')
      ylabel('hit distance')
      PlotClusterinResult([azim_set' hit_dist(:,1)],IDX_back(:,1));
      Color = hsv(max(IDX_back(:,1)));
       hold on
       pinch_bin_draw = [];
      for idx =1:max(IDX_back(:,1))
            cluster_idx = find(IDX_back(:,1)==idx);
            cut_dist = 0.7*max(hit_dist(cluster_idx,1)) + 0.3*min(hit_dist(cluster_idx,1));    
            pinch_bin_draw = [pinch_bin_draw cut_dist];
            plot([0 azim_set(end)],[cut_dist cut_dist],'Color',Color(idx,:),'LineStyle','--')
      end
      axis([0 azim_set(end) 0 max_ray+2])
      
      figure(fig_h);
      hold on 
      
      for pinchh = 1:max(IDX_back(:,1))
        draw_circle([ray_origin(1) ray_origin(2)],pinch_bin_draw(pinchh),[0 0 0],0,Color(pinchh,:));       
      end
      hold off
   end
   if max(IDX) == 1
    warning('clusterring warning : clustered altogether');
   end
   pinch_bin = [];
   
   % let't reject the cluster below the min_ray 
   valid_IDX = 1:max(IDX);
   for idx = 1:max(IDX)
       this_cluster = hit_dist_flat(IDX==idx);
       if max(this_cluster) < min_ray
           valid_IDX(idx) = [];
           warning('clusterring warning : the max val of this cluster < d_min. will be removed');
       end
   end
   
   % what are the representative dividing value of hit distance 
   for idx = valid_IDX
       % max is better cause it covers wide range but not much difference 
        pinch_bin=[pinch_bin (0.7*(max(hit_dist_flat(find(IDX==idx))) + 0.3*(min(hit_dist_flat(find(IDX==idx))))))];       
   end
   
   %% Phase 3: set DT    
   DT_set = {};
   pinch_bin = sort(pinch_bin);
   for i = 1:length(pinch_bin)
       pinch = pinch_bin(i);
       cast_res = double(hit_dist < pinch); 
       DT = signed_distance_transform([cast_res ; cast_res ; cast_res]); % periodic distance transform         
       DT = DT(N_azim+1:2*N_azim,:);  
       if sum(DT) == inf
            disp('No occlusion found. distance field will be returned as inf ')
       end
       DT_set{i} = DT;
   end

end