function sample_pnts = sample_around_edge(map,N_sample)
    % input : robotics occupancy grid  map object / number of sampled points
    occ_mat = map.occupancyMatrix;
    occ_mat(occ_mat>0.5) = 1; occ_mat(occ_mat<0.5) = 0; 

    dist_map=signed_distance_transform(occ_mat);

    % [r_edge,c_edge] = find(dist_map <= 0); % filled data
    [r_edge,c_edge] = find(dist_map == 0); % hull data
    sample_pnts = [];
    for i = 1:N_sample
        rand_idx = randi([1 length(r_edge)]);
        sample_pnt_center = map.grid2world([r_edge(rand_idx) c_edge(rand_idx)]);

        sample_min = sample_pnt_center- 1/map.Resolution/2*[1 1];    
        sample_max = sample_pnt_center +1/map.Resolution/2*[1 1];

        sample_pnt = sample_min + (sample_max-sample_min) .* rand(1,2); 
        sample_pnts = [sample_pnts ;sample_pnt];
        hh= plot(sample_pnt(1),sample_pnt(2),'bo','MarkerFaceColor','b');    
        hh.Color(4) = 0.2;
    end
    
end



% r_med = median(r_edge);
% c_med = median(c_edge);
% 
% 
% % % % selection window for realistic observation
% % r_edge=r_edge(floor(length(r_edge)/5):floor(length(r_edge)/2));
% % c_edge=c_edge(floor(length(c_edge)/5):floor(length(c_edge)/2));
% [~,r_idx_sorted] = sort(r_edge);
% [~,c_idx_sorted] = sort(c_edge);
% % I want to test L shape 
% select_window = intersect(r_idx_sorted(1:floor(length(r_idx_sorted)/1.5)),c_idx_sorted(1:floor(length(r_idx_sorted)/1.5)));
% r_edge = r_edge(select_window);
% c_edge = c_edge(select_window);