function fig_h= plot_raycast(map3,origin,hit_dist,azim_set,elev_set,Color,transp,LineWidth)
    % Color : string 
    fig_h = figure;
    hh = show(map3);
    alpha(0.2);
    hold on 
    title('')
    xlabel('')
    ylabel('')
    
    origin = reshape(origin,1,3);
    for azim_idx = 1:length(azim_set)
        for elev_idx = 1:length(elev_set)
            azim = azim_set(azim_idx); 
            elev = elev_set(elev_idx);
            ray_end = origin + hit_dist(azim_idx,elev_idx)*[cos(elev)*cos(azim) cos(elev)*sin(azim) sin(elev)];
            line = [origin ; ray_end];
            if(hit_dist(azim_idx,elev_idx) < max(hit_dist))
                plot3(ray_end(:,1),ray_end(:,2),ray_end(:,3),'rs','MarkerSize',10,'MarkerFaceColor','r');
            end
            hh= plot3(line(:,1),line(:,2),line(:,3),Color,'LineWidth',LineWidth);
            hh.Color(4) = transp;
        end        
    end
end