function I = pdf_tri_integral(mu,Sigma,triangle)
    % input :  mu - 1x2 / triangle : 2x3 
    j_axis = (triangle(:,2)-triangle(:,1))/norm(triangle(:,2) - triangle(:,1)); % j axis will be the p1 -> p2 
    i_axis = rot2(-pi/2)*j_axis;
    R01 = [i_axis j_axis];
    mu1 = R01'*mu'; % the mean w.r.t. the frame 1 
    Sigma1 = R01'*Sigma*R01;

    % triangle w.r.t frame1
    triangle1 = R01'*triangle;

    ymax = @(x) (triangle1(2,3) - triangle1(2,2))/(triangle1(1,3)-triangle1(1,2))*(x-triangle1(1,2)) + triangle1(2,2);  
    ymin = @(x) (triangle1(2,3) - triangle1(2,1))/(triangle1(1,3)-triangle1(1,1))*(x-triangle1(1,1)) + triangle1(2,1);

    pdf_fun=@(x1,x2) reshape(mvnpdf([x1(:) x2(:)],mu1',Sigma1),size(x1));
    I=integral2(pdf_fun,triangle1(1,3),triangle1(1,1),ymin,ymax)

end