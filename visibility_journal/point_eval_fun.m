function val_array = point_eval_fun(x1,x2,param)
    % this function is for numerical integration 
    % x1 : array / x2 : array 
    As=param.As;
    bs=param.bs;
    mu = param.mu;
    Sigma = param.Sigma;
    
    
    % if the point is in shade, returns 0 
    is_in_shading = zeros(size(x1(:)'));
    for n = 1:length(As)
        is_in_shading = logical(is_in_shading + all(As{n}*[x1(:)';x2(:)']<=bs{n}));        
    end
    % now, array
    is_in_shading = reshape(is_in_shading,size(x1));
    % this one also
    prob=reshape(mvnpdf([x1(:) x2(:)],mu,Sigma),size(x1));
    
    val_array = (1-is_in_shading).* prob;
%     val_array = prob;

   
end