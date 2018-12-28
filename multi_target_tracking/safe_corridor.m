function [vy,vz,ly,lz,V] = safe_corridor(pivot1,pivot2,max_stride,stride_step)
    % lx ly lz is the length of local axis     
    % ly,lz <= max_stride 
    
    vx = (pivot2 - pivot1); vx = vx/norm(vx);
    vy = [-vx(2) vx(1) 0]; vy = vy/norm(vy);
    vz = cross(vx,vy);
    
    % let's check by drawing 
%     hold on 
%     scatter3(pivot1(1),pivot1(2),pivot1(3),'ko');
%     scatter3(pivot2(1),pivot2(2),pivot2(3),'ko');
%     center = (pivot1 + pivot2)/2;    
%     
%     quiver3(center(1),center(2),center(3),vx(1),vx(2),vx(3),'k');        
%     quiver3(center(1),center(2),center(3),vy(1),vy(2),vy(3),'k');
%     quiver3(center(1),center(2),center(3),vz(1),vz(2),vz(3),'k');
%     
%     
%     hold off
%     
    expansion_dir = [1 0;0 1;-1 0; 0 -1]; % first col : vy / second col: vz
    
    
    for dir_idx = 1:length(expansion_dir)
        
    end
    
   
    
    
    
    
    

end