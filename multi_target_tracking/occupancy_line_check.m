function res=occupancy_line_check(map3,pnt1,pnt2,N_check)
    % this function investigates whether a line segment connecting pnt1 and
    % pnt2 hits the occupied region by checking "N_check" point on the line
    
    % output : returns false if is is not hit / 1 otherwise
    ckps = zeros(3,N_check);
    res = false;

    for i = 1:3 
           ckps(i,:) = linspace(pnt1(i),pnt2(i),N_check);           
    end
    
    for ck = 1:N_check
        if map3.checkOccupancy(ckps(:,ck)') == 1 % if unknown : -1  / free : 0 /occupied 1 
            res = true; 
            return 
        end
    end
    

end