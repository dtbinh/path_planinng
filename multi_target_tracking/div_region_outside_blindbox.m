
% this function divide the region outside of the blind box of target 1
% target 2

target1 = [2 4 0]'; target2 = [1 1 0]'; FOV = 2*pi/3;

% define the five facet excluding the bottom one 
% firstly find eight corner 

rel_dist =norm(target1 - target2); % distance between the targets  2D plane assumed 
blind_height = rel_dist/2/tan(FOV/2);

rotz90 = [0 -1 0 ; 1 0 0 ; 0 0 1];

vx = target2 - target1; vx = vx/norm(vx);
vy = rotz90 * vx;
vz = cross(vx,vy);

lcl_coord = [vx vy vz]; % check the research note
lx = rel_dist;
ly = 2*blind_height;
lz = 2*blind_height;
center = (target1 + target2)/2;
corners = center +[vx*lx vy*ly vz*lz]/2 * [ 1 -1 -1 1 1 -1 -1 1 ; 1 1 -1 -1 1 1 -1 -1 ; 1 1 1 1 -1 -1 -1 -1];


% check the corners 
figure                                              
hold on 
scatter3(corners(1,:),corners(2,:),corners(3,:),'ko');
scatter3(target1(1),target1(2),target1(3),'r*');
scatter3(target2(1),target2(2),target2(3),'r*');

% check the enclosing polyhedron 
axis equal
h = gcf;
axes = [h.CurrentAxes.XLim h.CurrentAxes.YLim h.CurrentAxes.ZLim ];

A = [vx' ; vy' ;-vx' ; -vy' ; vz' ; -vz']; % this is inward affine 
xs = corners(:,[1 2 3 4 1 5]); % points sequence matched with the normal surface vectors 
b= diag(A*xs);
plotregion(-A,-b);
axis(axes)
hold off


% let''s find the 8 outwardly divided regions 

A_out = -A; % outward affine 
b_out = -b;
affine_aug = [A_out b];

% sign = +1 for outward / -1 for inward
surf_select = {[1 2 5],[1 2 3 5],[2 3 5],[2 3 4 5],[3 4 5],[1 3 4 5],[1 4 5],[1 2 4 5] , ... % 1st floor 
    [1 2 5],[1 2 3 5],[2 3 5],[2 3 4 5],[3 4 5],[1 3 4 5],[1 4 5],[1 2 4 5]}; % 2nd floor

surf_direction = {[1 1 -1],[-1 1 -1 -1],[1 1 -1],[-1 1 -1 -1],[1 1 -1],[-1 -1 1 -1],[1 1 -1],[1 -1 -1 -1],... % 1st floor
     [1 1 1],[-1 1 -1 1],[1 1 1],[-1 1 -1 1],[1 1 1],[-1 -1 1 1],[1 1 1],[1 -1 -1 1]}; % 2nd floor 

 % the 16 divided regions
 A_blind_div = {};
 b_blind_div = {};
 figure
 title('division region')
 hold on 
 axis equal
 plotregion(-A,-b,[],[],[1 0 0],1);
scatter3(corners(1,:),corners(2,:),corners(3,:),'ko');
scatter3(target1(1),target1(2),target1(3),'r*');
scatter3(target2(1),target2(2),target2(3),'r*');


 for idx = 1:16
     A_blind_div{idx} = surf_direction{idx}' .* A_out(surf_select{idx},:);
     b_blind_div{idx} = surf_direction{idx}'.* b_out(surf_select{idx});         
     [~,~,flag]=linprog([],A_blind_div{idx},b_blind_div{idx});
     if(flag == -2)
        warning('infeasible space')
     end
     plotregion(-A_blind_div{idx},-b_blind_div{idx},axes(1:2:end),axes(2:2:end),[],0.4)  
     axis(axes)
 end
 
 hold off
 
 %% 
 figure 
 [As_no_blind,bs_no_blind]=no_blind_region (target1,target2,FOV);
 hold on 
 
  for idx = 1:17
     [~,~,flag]=linprog([],As_no_blind{idx},bs_no_blind{idx});
     if(flag == -2)
        warning('infeasible space')
     end
     plotregion(-As_no_blind{idx},-bs_no_blind{idx},[xl yl zl],[xu yu zu],[],0.4)  
     axis([xl xu yl yu zl zu])
 end
 
 
 
 
 
 
 
 




