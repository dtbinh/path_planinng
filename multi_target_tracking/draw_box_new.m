function draw_box_new(rect_center,R,geo_rect,color_str,alpha)
% Dependencies : plot region 
% geo_rect : half of total length in each direction 
addpath('..\plotregion\');

rect_center = reshape(rect_center,3,[]);

vx = R(:,1);
vy = R(:,2);
vz = R(:,3);

lx = geo_rect(1); ly = geo_rect(2); lz = geo_rect(3);

corners = rect_center +[vx*lx vy*ly vz*lz] * [ 1 -1 -1 1 1 -1 -1 1 ; 1 1 -1 -1 1 1 -1 -1 ; 1 1 1 1 -1 -1 -1 -1];

A = [vx' ; vy' ;-vx' ; -vy' ; vz' ; -vz']; % this is inward affine 
xs = corners(:,[1 2 3 4 1 5]); % points sequence matched with the normal surface vectors 
b= diag(A*xs);


plotregion(-A,-b,[],[],color_str,alpha)


end