%% Description 
% this script is used for illustrating the measure for visibility using
% SEDT 

%% Probelm settings
% target position 
target=[5 6]'; tracker_position=[2.5 4];

% basic parameters 
dim=2;  w_v0=10; N_stride=20; N_azim=51; max_ray_length=3; sample_ray_length=[ 3 2];

% map scale 
ws_range=[0 10 ; 0 10];


%% SCENARIO1 : COMPLEX OBSTACLE CONFIG
% Target position 
target_pos=[6 5]; 

% Obstacles 
T1=SE2; T2=SE2; T3=SE2; 
T1.t=[6 7]'; T2.t=[6 3]; T3.t=[8.3 5.7];
obs1_scale=[2 0.3];  obs2_scale=[2 0.3];  obs3_scale=[0.3 1.5];
obs1=obstacle2(T1,obs1_scale); obs2=obstacle2(T2,obs2_scale); obs3=obstacle2(T3,obs3_scale); 

% ASAP objects
asap=ASAP_problem(dim,ws_range,{obs1,obs2,obs3},w_v0,N_stride,N_azim,max_ray_length,sample_ray_length,[]);
cast_result=asap.cast_ray(target_pos);

% P-SEDT 
cast_result_binary=cast_result<max_ray_length;
periodic_cast_result_binary=repmat(cast_result_binary,1,3);
D=signed_distance_transform(periodic_cast_result_binary);
D=D(N_azim+1:2*N_azim); 


figure
asap.mapplot % obstacle 
hold on
plot(target_pos(1),target_pos(2),'ro')
thetas=linspace(0,2*pi,N_azim);
clr={'b','g','r'};
for i=1:N_azim
    theta=thetas(i);
    end_pnt=target_pos+cast_result(i)*[cos(theta) sin(theta)];
    if cast_result_binary(i)
        plot([target_pos(1) end_pnt(1)],[target_pos(2) end_pnt(2)],'k')
    else
        plot([target_pos(1) end_pnt(1)],[target_pos(2) end_pnt(2)],'r')
    end
end

%% 
figure 
title('Visibilty')
polarplot(thetas,D/max(D),'k')
rlim([-0.5 1])
% rticks([0 3 10])
% rticklabels({'V = 0 ','V = 3','V= 10'})
% 



