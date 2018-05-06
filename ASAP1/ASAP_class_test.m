%% Probelm settings
% target position 
target=[5 6]'; tracker_position=[2.5 4];
target_path=[5 6.5 ; 6 6.5 ; 6.5 6.5];

% basic parameters 
dim=2;  w_v0=2; N_stride=15; N_azim=25; max_ray_length=3; sample_ray_length=[4 3 2];

% map scale 
ws_range=[0 10 ; 0 10];

% set obstacle shape and position
T1=SE2; T2=SE2;
T1.t=[5 8]'; T2.t=[5,5.3];
obs1_scale=[3 0.5];  obs2_scale=[1 0.3];
obs1=obstacle2(T1,obs1_scale); obs2=obstacle2(T2,obs2_scale);


% ASAP class 
asap=ASAP_problem(dim,ws_range,{obs1,obs2},w_v0,N_stride,N_azim,max_ray_length,sample_ray_length);
figure
asap.mapplot
hold on
plot(target_path(:,1),target_path(:,2),'r*-');
plot(tracker_position(1),tracker_position(2),'ko','LineWidth',2);

%% Graph construction 
% [layer_names,layer_poses,layer_vises]=asap.layer_calc(target,1);
asap.graph_init(tracker_position);
asap.graph_insertion(target_path(1,:));
asap.graph_insertion(target_path(2,:));
asap.graph_insertion(target_path(3,:));
asap.graph_wrapper();
path=asap.path_proposal();

% asap.mapplot
shapes={'s','*','d'};

for i=1:length(asap.layer_info_pose)
    for j=1:length(asap.layer_info_pose{i})
        candid_pose=asap.layer_info_pose{i}{j};
        
        plot(candid_pose(1),candid_pose(2),strcat('k',shapes{i}),'MarkerSize',6)
    end
end

plot([tracker_position(1); path(:,1)],[tracker_position(2) ;path(:,2)],'gs-','MarkerSize',6);



