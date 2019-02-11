function res = is_within_box(rects,T_rects,eval_point,tol)
    % rects : list of rect (2x4) / T_rects : the transformation matrix from
    % world to the local frame 
    res = false;
    for n = 1:length(rects)
        lcl_pnts = T_rects{n}.inv*rects{n};
        eval_pnt_lcl = T_rects{n}.inv*[eval_point(1); eval_point(2)];
        xl = min(lcl_pnts(1,:)); xu = max(lcl_pnts(1,:));
        yl = min(lcl_pnts(2,:)); yu = max(lcl_pnts(2,:));
        res=(xl-tol<eval_pnt_lcl(1)) && (xu+tol>eval_pnt_lcl(1)) && (yl-tol<eval_pnt_lcl(2)) && (yu+tol>eval_pnt_lcl(2)) ;
        if res
            break
        end
    end


end