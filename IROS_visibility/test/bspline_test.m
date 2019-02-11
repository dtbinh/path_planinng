        t = [0.1 0.1 0.1 0.1 0.3 0.4 0.5 0.6 0.7 0.8 0.9 2 2 2 2];
        % control points (unknown)
        D_0 = [ 0.4965 0.6671 0.7085 0.6809 0.2938 0.1071 0.3929 0.5933 0.8099 0.8998 0.8906 ...
              ; 0.8436 0.7617 0.6126 0.212 0.1067 0.3962 0.5249 0.5015 0.3991 0.6477 0.8553 ];
        % points on B-spline curve
        k = 4;
        [M_0,x] = bspline_deboor(k,t,D_0);
        
        figure 
        hold on
        plot(D_0(1,:), D_0(2,:), 'go-');
        plot(M_0(1,:), M_0(2,:), 'b');
        
        