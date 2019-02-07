function [A_shading,b_shading] = get_shading(observer,rect)
    % observer : 1x2 / rects : 2x4 pnts matrix 
    % the shading region representing the shading region 
    pnt1 = rect(:,1); pnt2 = rect(:,2); pnt3 = rect(:,3); pnt4 = rect(:,4);
    box_center = (pnt1 + pnt2 + pnt3 + pnt4)/4;
    bearing_vec2_box = [box_center(1)- observer(1);box_center(2)- observer(2)];
    % lambda function smeasuring two inner angle between two vectors 
    angle_btw_two = @(x1,x2) acos(abs(dot(x1,x2))/norm(x1)/norm(x2));
    % let;s compare 4 rays 
    ray1 = pnt1 - observer'; theta1 = angle_btw_two(ray1,bearing_vec2_box);
    ray2 = pnt2 - observer'; theta2 = angle_btw_two(ray2,bearing_vec2_box);
    ray3 = pnt3 - observer'; theta3 = angle_btw_two(ray3,bearing_vec2_box);
    ray4 = pnt4 - observer'; theta4 = angle_btw_two(ray4,bearing_vec2_box);
    pnts = [pnt1 pnt2 pnt3 pnt4];
    rays = [ray1 ray2 ray3 ray4];
    angles = [theta1 theta2 theta3 theta4];
    [~,sort_idx] = sort(angles); 
    proj_pnts = pnts(:,sort_idx(3:4));
    for i = 1:2
        h = plot([proj_pnts(1,i) observer(1)],[proj_pnts(2,i) observer(2)],'LineWidth',1,'Color',[0.8 0 0]);
        h.Color(4) =0.4;
    end
    
    % determine linear inequality 
    % direction of n1-n2 ? let's determine the sign 
    n1 = rot2(pi/2)*(proj_pnts(:,1) - observer'); 
    n2 = rot2(pi/2)*(proj_pnts(:,2) - observer');
    n3 = rot2(pi/2)*(proj_pnts(:,2) - proj_pnts(:,1));
    A_init = [n1';n2']; b_init = [dot(n1,observer) ; dot(n2,observer)];
    correct_flag = false;
    for n2_sign = [-1 1]
        for n1_sign = [-1 1]
            A = diag([n1_sign n2_sign])*A_init;
            b = diag([n1_sign n2_sign])*b_init;
            if all(A*(proj_pnts(:,1)+proj_pnts(:,2))/2< b)            
                    correct_flag = true;
                break
            end
        end
        if correct_flag
            break
        end
    end

    
    bearing_vec2_two_pnts = (proj_pnts(:,2) + proj_pnts(:,1))/2 - [observer(1) ; observer(2)];
    if dot(n3,bearing_vec2_two_pnts)>0
        n3 = -n3;
    end
    
    A =[A;n3']; b = [b;dot(n3,proj_pnts(:,1))];
    A_shading = A;
    b_shading = b;

end