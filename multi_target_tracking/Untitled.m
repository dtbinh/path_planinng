target1 = [2 2 0];
target2 = [3 5 0];

FOV = 2*pi/3;
is_in_blind3(target1,target2,FOV,[],1)


rel_dist = norm(target1 - target2); % distance between the targets 
blind_height = rel_dist/2/tan(FOV/2);

% inspection = [2.5 3.5 1.58];
inspection = [2.5 7/2 blind_height];

hold on
scatter3(inspection(1),inspection(2),inspection(3),'g*')
plot3([target1(1) target2(1)],[ target1(2) target2(2)],[blind_height blind_height],'b-','LineWidth',2)
v1 = target1 -inspection;
v2 =target2 -inspection;
ang = acos((dot(v1,v2))/norm(v1)/norm(v2))*180/pi;
axis equal