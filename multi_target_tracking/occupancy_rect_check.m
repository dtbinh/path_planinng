function res=occupancy_rect_check(map3,rect_center,R_rect,geo_rect,is_plot_rect)
    % rect_center : the center of rectangle
    % R_rect : pose of the rect 
    % geo_rect = [lx ly ly] : half of tatol length in each direction of
    % rectangle 
    % is_plot_rect : should I draw a rectangle 
    
    rect_center = reshape(rect_center,3,[]); % vector shape 3 x 1 
    
    
    lx = geo_rect(1);
    ly = geo_rect(2);
    lz = geo_rect(3);
    
    % local coordinate     
    ix = R_rect(:,1);
    iy = R_rect(:,2);
    iz = R_rect(:,3);
    
        
    N_check_x = 10;
    N_check_y = N_check_x * ly/lx;
    N_check_z = N_check_x * lz/lx;
    
    res = false;
    
%     if is_plot_rect
%         pnt_lower = rect_center  - R_rect * [lx ly lz]';
%         pnt_upper = rect_center + R_rect * [lx ly lz]';
%         draw_box(pnt_lower,pnt_upper,'k',0.5);        
%     end

    hold on
    for ly_mesh = linspace(-ly,ly,N_check_y)
        for lz_mesh = linspace(-lz,lz,N_check_z)
            pnt1 = rect_center + lx*ix + ly_mesh * iy + lz_mesh * iz;
            pnt2 = rect_center -  lx*ix + ly_mesh * iy + lz_mesh * iz;    
            pnts = [pnt1 pnt2];
            if is_plot_rect
                plot3(pnts(1,:),pnts(2,:),pnts(3,:),'ro-')
            end
            if occupancy_line_check(map3,pnt1,pnt2,N_check_x)            
                if is_plot_rect
                    plot3(pnts(1,:),pnts(2,:),pnts(3,:),'go-','LineWidth',3)
                    hold off
                end                
                res = true;
                return 
            end
        end               
    end
    
    hold off 


end