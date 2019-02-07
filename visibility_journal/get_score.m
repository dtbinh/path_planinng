function score = get_score(observer,mu,Sigma,As,bs,alpha)
    % input : mu,Sigma - for pdf / As,bs - the shading region / alpha :
    % 100(1-alpha) HDR
    
    % Note : this method does not guarantee the integration performance due
    % to the discontinuity of the function 
    
    [eigvec,eigval] = eig(Sigma);
    
    R = eigvec;
    sigma1 = sqrt(eigval(1,1));
    sigma2 = sqrt(eigval(2,2));
    
    chi_val = -2*log(alpha);
    
    param.As = As;
    param.bs = bs;
    param.mu = mu;
    param.Sigma = Sigma;
    
    fun_xy_trans = @(x1_,x2_) point_eval_fun(sigma1*(R(1,1).*x1_+R(1,2).*x2_)+mu(1),sigma2*(R(2,1)*x1_ + R(2,2)*x2_)+mu(2),param)*sigma1*sigma2; % linear transform 
    fun_polar = @(r,theta) fun_xy_trans(r.*cos(theta),r.*sin(theta)).*r;
    score = integral2(fun_polar,0,sqrt(chi_val),0,2*pi);   
    
    

end