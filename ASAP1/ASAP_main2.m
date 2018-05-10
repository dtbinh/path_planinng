%% simulation env 
% this includes the 4 scenarios all together  

% map scale 
ws_range=[0 25 ; 0 15];
tracker_position=[2.5 4];

T1=SE2; T2=SE2; T3=SE2; T4=SE2;  
T1.t=[11 10]'; T2.t=[23 11]; T3.t=[7 5]; T4.t=[14 3];
obs1_scale=[8 3];  obs2_scale=[0.5 2];  obs3_scale=[1 0.5]; obs4_scale=[1 0.5];
obs1=obstacle2(T1,obs1_scale); obs2=obstacle2(T2,obs2_scale); obs3=obstacle2(T3,obs3_scale); obs4=obstacle2(T4,obs4_scale); 

% basic parameters 
dim=2;  w_v0=4; N_stride=20; N_azim=25; max_ray_length=3; sample_ray_length=[4 3 2];

asap=ASAP_problem(dim,ws_range,{obs1,obs2,obs3,obs4},w_v0,N_stride,N_azim,max_ray_length,sample_ray_length);
asap.mapplot
hold on

% target path
T_step=10;
target_path_x=[ linspace(5,19,T_step)  20*ones(1,T_step-2) ]';
target_path_y=[  6*ones(1,T_step) linspace(7,13,T_step-2)  ]';
target_path=[target_path_x target_path_y];
plot(target_path(:,1),target_path(:,2),'rs-');
plot(tracker_position(1),tracker_position(2),'k^','LineWidth',2);% asap.mapplot
tracker_vel=[0 0];
% history of position and velocity of tracker 
N_step=length(target_path);

%% variables 
follow_ratio=4/5;

%% simulation
hold on 
generate_cond=true;
for t_step=2: N_step
        if generate_cond
       %% predict & generate path 
        N_pred=3; % horizon 
        prev_target_pos=target_path(t_step-1,:);
        cur_target_pos=target_path(t_step,:);        
        dx=cur_target_pos(1)-prev_target_pos(1);
        dy=cur_target_pos(2)-prev_target_pos(2);        
        asap.graph_init(tracker_position);               
        for horz=1:N_pred
            asap.graph_insertion(cur_target_pos+[dx*horz dy*horz])
        end                
        asap.graph_wrapper();       
        waypoint=asap.path_proposal(); 
        N_iter=50;
        plot(waypoint(:,1),waypoint(:,2),'ms','MarkerSize',12,'LineWidth',2)
        [xs_gen,vs_gen]=asap.PM.traj_gen(tracker_position,tracker_vel,waypoint,N_iter);            
        
        %% follow path for given time 
        N_gen=floor(length(xs_gen)*follow_ratio);
        plot(xs_gen(1:N_gen,1),xs_gen(1:N_gen,2),'g-','LineWidth',4)
        tracker_position=xs_gen(N_gen,:); % after following, we are here
        tracker_vel=vs_gen(N_gen,:);        
        end
        
        if mod(t_step,4)==0
           generate_cond=true;               
        else
            generate_cond=false;               
        end
        
end
%% plot 

figure
asap.mapplot % obstacle 
hold on
plot(xs(:,1),xs(:,2),'g-','LineWidth',4)

plot(target_path(:,1),target_path(:,2),'r*-');
plot(tracker_position(1),tracker_position(2),'ko','LineWidth',2);% asap.mapplot

% candidates 
shapes={'s','d','*','x'};

for i=1:length(asap.layer_info_pose)
    for j=1:length(asap.layer_info_pose{i})
        candid_pose=asap.layer_info_pose{i}{j};        
        plot(candid_pose(1),candid_pose(2),strcat('k',shapes{i}),'MarkerSize',6)
    end
end

plot([tracker_position(1); path(:,1)],[tracker_position(2) ;path(:,2)],'ms','MarkerSize',12,'LineWidth',2);




