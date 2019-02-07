function [rect_pnts,T_min] = box_fit(sample_pnts)
    % input : points = N x 2
    % output: rect_pnts = 4 x 2 / transformation matrix world to body 
    
    centroid = mean(sample_pnts);
    N_res = 6;
    S_min = 1e+6;
    
    for theta = linspace(0,pi/2,N_res)
        % find the fitting box 
        coeff = [sin(theta) cos(theta) ; -cos(theta) sin(theta)];
        T = SE2(coeff);
        T.t = centroid;
        sample_pnts_tran=(T.inv)*sample_pnts';
        % h_tran= plot(sample_pnts_tran(1,:),sample_pnts_tran(2,:),'co','MarkerFaceColor','c');    
        % h_tran.Color(4) = 0.5;
%         axis([0 10 0 10])
        upper_pnt = max(sample_pnts_tran')';
        lower_pnt = min(sample_pnts_tran')';
        S = (upper_pnt(1)-lower_pnt(1))*(upper_pnt(2)-lower_pnt(1));
        if  S < S_min   
            S_min = S;
            pnt3 = T*upper_pnt;
            pnt1 = T*lower_pnt;
            pnt2 = T*[upper_pnt(1); lower_pnt(2)];
            pnt4 = T*[lower_pnt(1) ; upper_pnt(2)];
            upper_pnt_min = upper_pnt;
            lower_pnt_min = lower_pnt;
            T_min = T;
        end

    end
    
    % ccw 
    rect_pnts = [pnt1 pnt2 pnt3 pnt4];
    
end
    
    