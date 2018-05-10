classdef path_manager < handle 
    %% properties 
    properties 
        px; 
        py;
        obstacles;   % cell of obstacle    
    end
    %% methods 
    methods        
        function obj=path_manager(obstacles)
            obj.obstacles=obstacles;
            obj.px={};
            obj.py={};            
        end    
        %% path generation  ( poly_traj_gen needed ! ) 
        function [xs,vs]=traj_gen(path_manager,x0,xdot0,waypoints,N_iter)            
            % this function generate trajectory. the polynomial coeff can
            % be found in path_manager (px py) in form of cell 
            % x0, xdot0 ( 1 x 2) / waypoints ( N x 2)
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % TUNEING : polynomial order (n) / max_iter for optimization /
            % bounding value of vset 
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
            % parsing 
            N_waypoints=size(waypoints,1);
            waypoint={}; % waypoint is cell (~= waypoints)
            for i=1:N_waypoints
                waypoint{i}=waypoints(i,:);
            end
                      
            n=7; % poly order 
            vset=(waypoint{1}-x0)';
            for i=2:length(waypoint)
                vset=[vset ; waypoint{i}(1)-waypoint{i-1}(1) ; waypoint{i}(2)-waypoint{i-1}(2) ];
            end

            % function handle
            sum_cost=@(v) poly_traj_gen(v,n,x0,xdot0,waypoint,path_manager);

            %%%%%% two-stage optimization %%%%%%%%%%%%%%%%%
            % inner : fast QP 
            % outer : non linear optimization for less cost using
            % connecting velocity 
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
            % bound of velocity
            upper_limit=1.5;
            lower_limit=-1.5;
            len=length(vset);

            options = optimoptions('fmincon','Algorithm','SQP','MaxFunctionEvaluations',N_iter);
            fmincon(sum_cost,vset,[],[],[],[],lower_limit*ones(len,1),upper_limit*ones(len,1),[],options);  
            
            xs=[];
            vs=[];
            % for each segment, we construct the path 
            for seg=1:length(path_manager.px)
                px_seg=path_manager.px{seg};
                py_seg=path_manager.py{seg};    
                n_poly=length(px_seg)-1;
                t=linspace(0,1,20);
                x=zeros(length(t),1);
                y=x; vx=y; vy=vx;
                for i=1:length(t)
                    t_=t(i);
                    x(i)=px_seg'*path_manager.t_vec(n_poly,t_,0);    
                    y(i)=py_seg'*path_manager.t_vec(n_poly,t_,0);
                    vx(i)=px_seg'*path_manager.t_vec(n_poly,t_,1);
                    vy(i)=py_seg'*path_manager.t_vec(n_poly,t_,1);                                        
                end                                                                     
                xs=[xs ; [x y]];
                vs=[vs ; [vx vy]];
            end      
            

        end
        
        
        %% utils 
        % time vector for fast computation of polynomial 
        function vec=t_vec(~,n,t,n_diff)
            % this computes time vector.
            % n: order of poly
            % t: eval point 
            % n_diff : diff order 
            
            vec=zeros(n+1,1);
            switch n_diff
                case 0
                    for i=1:n+1
                        vec(i)=t^(i-1);
                    end            
                case 1
                    for i=2:n+1
                        vec(i)=(i-1)*t^(i-2);
                    end
                    
                case 2
                    for i=3:n+1
                        vec(i)=(i-1)*(i-2)*t^(i-3);
                    end                       
            end        
        end
        
        function path_plot(path_manager)               
            % for each segment, we plot the path 
            for seg=1:length(path_manager.px)
                px_seg=path_manager.px{seg};
                py_seg=path_manager.py{seg};    
                n_poly=length(px_seg)-1;
                t=linspace(0,1,20);
                x=zeros(length(t),1);
                y=x;
                for i=1:length(t)
                    t_=t(i);
                    x(i)=px_seg'*path_manager.t_vec(n_poly,t_,0);    
                    y(i)=py_seg'*path_manager.t_vec(n_poly,t_,0);
                end                
                plot(x,y,'gs-','LineWidth',2);                               
            end                  
            
        end
        
        
        function mapplot(path_manager)
            % this funnction plot the current problem settings
            hold on 
                       
            for i=1:length(path_manager.obstacles)
                obs=path_manager.obstacles{i};
                obs.plot
            end
            
            hold off
        end
        
        function map=obs_inflation(path_manager,lower_left_corner,upper_right_corner,Nx,Ny)
            % corner : 1 x 2
            % for A* local planner , computes inflated binary occupancy map
            % in the boundary of box defined by two corner 
            % (i,j) elem of map = lower_left + (dx/2 + (i-1) * dx ,dy/2+  (j-1) * dy )
            % Nx, Ny normally 15
            
            map=zeros(Nx,Ny);
            dx=(upper_right_corner(1)-lower_left_corner(1))/Nx;            
            dy=(upper_right_corner(2)-lower_left_corner(2))/Nx;
                        
            for ix=1:Nx
                for iy=1:Ny
                    cur_pos=lower_left_corner+ [dx/2+(ix-1)*dx dy/2+(iy-1)*dy ];
                    
                    % obstacle check 
                    for obs_idx=1:length(path_manager.obstacles) % for obstacle list
                        this_obs=path_manager.obstacles{obs_idx};
                        if this_obs.isobs(cur_pos')
                            map(ix,iy)=1;
                            break
                        end                        
                    end  % for all obstacle
                    
                end                               
            end 
                            
           SEDT=signed_distance_transform(map); 
           % threshold should be turned 
           map=(SEDT<3);               
                     
        end
                
        function waypoints=guidance_waypoint(path_manager,x0,xf,lower_left_corner,upper_right_corner,Nx,Ny,N_pnts)
            % this function calculate waypoints that derive the path to
            % avoid obstacle region
            % box should be larget than acutal BB including the x0, xf 
            % N_pnts : how many guidance point 
            % waypoints : N_pnts x 2
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % TUNEING : bedding size             
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
            
            waypoints=[];            
            inflated_map=path_manager.obs_inflation(lower_left_corner,upper_right_corner,Nx,Ny);            
            if sum(sum(inflated_map))==0
                return % no obstacle found!
            end
            local_path=A_star(inflated_map,x0,xf,lower_left_corner,upper_right_corner);
            path_len=length(local_path);
            guidance_idx=floor(linspace(1,path_len,N_pnts+2));
            guidance_idx=guidance_idx(2:end-1);
            waypoints=local_path(guidance_idx,:);                   
        end           
            
       end
          
end