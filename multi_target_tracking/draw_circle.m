function draw_circle(xc,r,color,alpha,edge_color)
     % this function draw a circle sector 
     % xc : center of circle 
     % theta_min, theta_max : range of angle 
     % r : radius 
     % color : string 
     
     N = 30 ;
     theta_max = 2*pi;
     theta_min = 0;
     theta_d = (theta_max - theta_min)/N;     
     xs = [];
     ys = [];
     
     for theta = theta_min : theta_d : theta_max
         X_cur = xc + r * [cos(theta) sin(theta)];
         xs = [xs X_cur(1)];
         ys = [ys X_cur(2)];         
     end
     
%      patch(xs,ys,color,'FaceAlpha',alpha,'EdgeColor','k');
     patch(xs,ys,color,'FaceAlpha',alpha,'EdgeColor',edge_color);


end