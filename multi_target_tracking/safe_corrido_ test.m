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
    vert2 = v_div{h}{idx_seq(h)};    
    vert = [vert1 ; vert2];       
    K = convhull(vert(:,1), vert(:,2),vert(:,3));    
    shp = alphaShape(vert2(:,1),vert2(:,2),vert2(:,3),2);
    plot(shp,'EdgeColor','g','FaceColor',[0 1 0],'FaceAlpha',0.5,'LineWidth',1)
    
    shp_aug = alphaShape(vert(K,1),vert(K,2),vert(K,3),100);
    plot(shp_aug,'EdgeColor','none','FaceColor',[0 0 0],'FaceAlpha',0.5,'LineWidth',1,'MarkerEdgeColor','none')


    
    [A_corr,b_corr]=vert2con(vert(K,:));
    % corridor connecting each waypoint polygon 
    
%     corridor_polygon_seq{h}.A =[A_corr] ;
%     corridor_polygon_seq{h}.b = [b_corr]; 
    
    corridor_polygon_seq{h}.A =[A_corr ] ;
    corridor_polygon_seq{h}.b = b_corr;        
    axis([xl xu yl yu zl zu])
end

% save('polygon_seq','waypoint_polygon_seq','corridor_polygon_seq');

%% Phase 9




