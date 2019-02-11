function integral_val=eval_visibility(XLimit,YLimit,rects,sensor_model,alpha,mu,Sigma,observer)
    % 2019/2/7 written by JBS  visibility metric 
    % rects = the boxes from fitting / alpha = (1-alpha)100 HDR/ mu Sigma is pdf of target / observer = position of observer  
    % sensor model = struct which has FOV and range as its element 
  
    
    % (1-alpha)100 inscrbing box 

    [eigvec,eigval] = eig(Sigma);
    
    R = eigvec;
    T_elip = SE2(R);
    T_elip.t = mu;
    
    sigma1 = sqrt(eigval(1,1));
    sigma2 = sqrt(eigval(2,2));
    
    chi_val = -2*log(alpha);
    
    r1 = sqrt(chi_val)*sigma1;
    r2 = sqrt(chi_val)*sigma2;
    integration_window_pnts = T_elip*[-r1 r1 r1 -r1;-r2 -r2 r2 r2];


   % shading region
    shades_poly = {};
    for n = 1:length(rects)
        [A,b]=get_shading(observer,rects{n});
        A=[A;eye(2) ; -eye(2)]; b = [b ; XLimit(2) ; YLimit(2) ; XLimit(1) ;YLimit(1)];     
        % this could be replaced with others
        vertices=lcon2vert(A,b);
        hull_idx=convhull(vertices);
        shade_poly{n}=polyshape(vertices(hull_idx,1),vertices(hull_idx,2));
    end
    
    
    % sensor model 
    FOV = sensor_model.FOV;
    range = sensor_model.range;
    bearing_vec = (mu - observer)';
    bearing_vec = bearing_vec/norm(bearing_vec);
    sensor_triangle = observer';
    sensor_triangle = [sensor_triangle observer'+bearing_vec*range+rot2(pi/2)*bearing_vec*range*tan(FOV)];
    sensor_triangle = [sensor_triangle observer'+bearing_vec*range-rot2(pi/2)*bearing_vec*range*tan(FOV)];
    % patch(sensor_triangle(1,:),sensor_triangle(2,:),'g','FaceAlpha',0.05);

    % integral domain decision     
    pg_sensor = polyshape(sensor_triangle(1,:),sensor_triangle(2,:));
    pg_HDR = polyshape(integration_window_pnts(1,:),integration_window_pnts(2,:));
    tic 
    pg_avaliable = intersect(pg_sensor,pg_HDR);

    % occlusion culling 
    for n = 1:length(rects)
        pg_avaliable = subtract(pg_avaliable,shade_poly{n});
    end
    
    
    % triangulaion and integration 
    
    integral_val = 0;
    if isempty(pg_avaliable.Vertices)
        return 
    else
        TRI = delaunay(pg_avaliable.Vertices);
        for r = 1:size(TRI,1)
            mesh = pg_avaliable.Vertices(TRI(r,:),:);
            integral_val = integral_val + pdf_tri_integral(mu,Sigma,mesh');
        end
    end

end