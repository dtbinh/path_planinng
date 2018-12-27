%% Phase 8 : plotting the planned path of polyhedron 

figure
show(map3)
hold on 

for h = 1:H    
     if h == 1 
            plot3(tracker(1),tracker(2),tracker(3),'mo','MarkerFaceColor','m')
     end
     
    plotregion(-A_div{h}{idx_seq(h)} ,-b_div{h}{idx_seq(h)} ,[xl yl zl]',[xu yu zu]',[1,0,1],0);
    hold on
    % 2D version 
%     waypoint_polygon_seq{h}.A = A_div{h}{idx_seq(h)};
%     waypoint_polygon_seq{h}.b = b_div{h}{idx_seq(h)};
    
    % 3D version 
    waypoint_polygon_seq{h}.A = [A_div{h}{idx_seq(h)}];
    waypoint_polygon_seq{h}.b =[b_div{h}{idx_seq(h)}];
               
    if h ==1 
        vert1 = tracker';
    else
        vert1 = v_div{h-1}{idx_seq(h-1)};        
    end
    
    Nk1 = size(vert1,1);
    Nk2 = size(vert2,1);
    
    vert2 = v_div{h}{idx_seq(h)};    
    vert = [vert1 ; vert2];       
    K = convhull(vert(:,1), vert(:,2),vert(:,3),'simplify', true);    
          
    shp = alphaShape(vert2(:,1),vert2(:,2),vert2(:,3),2);
    plot(shp,'EdgeColor','g','FaceColor',[0 1 0],'FaceAlpha',0.5,'LineWidth',1)
    
    % This draw a connecting convex hull 
    shp_aug = alphaShape(vert(K,1),vert(K,2),vert(K,3),100);
    plot(shp_aug,'EdgeColor','none','FaceColor',[0 0 0],'FaceAlpha',0.5,'LineWidth',1,'MarkerEdgeColor','none')
    
    % identify the owener of each point in convex hull (does it belong to vert1 or vert2?)
    K_conv=unique(reshape(K,1,[])); % indices of vertex which is not insider 
    
    K_conv1=intersect(K_conv,1:Nk1);
    K_conv2=setdiff(K_conv,K_conv1);
    
    conv_vert1 = vert(K_conv1,:);
    conv_vert2 = vert(K_conv2,:);
    
    % nextly, we enumerate the combination(pair) of convex vertecies 
    for K_conv1_idx = K_conv1
        for K_conv2_idx = K_conv2
            % here is where the expansion comes in 
            pivot1 = vert(K_conv1_idx,:);
            pivot2 = vert(K_conv2_idx,:);
            % first check if this line segment is free 
            line = [pivot1 ; pivot2]; % 2 x 3 
            % this gurantees the safety of line segment 
            if ~occupancy_line_check(map3,line(1,:),line(2,:))
                % let's find box expansion 
                
                
            end
%             plot3(line(:,1),line(:,2),line(:,3),'m-')
            
        end
    end
    
    

    
    [A_corr,b_corr]=vert2con(vert(K,:));
    % corridor connecting each waypoint polygon 
    
%     corridor_polygon_seq{h}.A =[A_corr] ;
%     corridor_polygon_seq{h}.b = [b_corr]; 
    
    corridor_polygon_seq{h}.A =[A_corr ] ;
    corridor_polygon_seq{h}.b = b_corr;        
    axis([xl xu yl yu zl zu])
end

% save('polygon_seq','waypoint_polygon_seq','corridor_polygon_seq');

%% Phase 8-1 






