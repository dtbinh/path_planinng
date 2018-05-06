function [p,residual]=target_traj_fitting(t_obs,x_obs,n,weights,lambda,obs_period,pred_period)
%% input
    % t_obs : obs time (global clock)
    % weights : fitting weight
    % n : order of polynomial 
    % x_obs : obs_data
    % lambda : optimization ratio of accel
    % obs_period : how long we observed (global clock)
    % pred_period : how long we predict (global clock)
    
    
%% Output 
    % 0th order to Nth order
    
%% time scaling 
    % squeeze t_obs to [0 t_pred_start]
    t_obs=(t_obs-t_obs(end))/(t_obs(end)-t_obs(1))*obs_period/pred_period;
%% time integral matrix

  % Compute the time integral matrix from 0 to 1
    T0=zeros(n+1,n+1);     
    T1=zeros(n+1,n+1);  % intergral of time vector 1st derviative    
    T2=zeros(n+1,n+1);  % integral of timevector 2nd derviative 
    T3=zeros(n+1,n+1);  % jerk 
    for i=1:n+1
        for j=1:n+1
            T0(i,j)=1/(i+j-1);
            
            if i>1 && j>1
                T1(i,j)=(i-1)*(j-1)/(i+j-3);
            end
            
            if i>2 && j>2
                T2(i,j)=factorial(i-1)/factorial(i-3)*factorial(j-1)/factorial(j-3)/(i+j-6+1);
            end
            
            
            if i>3 && j>3
                T3(i,j)=factorial(i-1)/factorial(i-4)*factorial(j-1)/factorial(j-4)/(i+j-8+1);
            end
        end
    end


    
%% Quadform  pQp' + H p 
Q=lambda*T2;
H=0;
for i=1:length(t_obs)
   Q=Q+weights(i)*t_vector(t_obs(i),0,n)*t_vector(t_obs(i),0,n)';
   H=H-2*weights(i)*x_obs(i)*t_vector(t_obs(i),0,n)';
end

%% solve with quad prog 
options=optimoptions('quadprog','Display','off');
% polynomial eval at [0 t_pred]
p=(quadprog(2*Q,H,[],[],[],[],[],[],[],options));
residual=p'*(Q-lambda*T2)*p+H*p+sum(weights.*(x_obs.^2));
% 
% p=flipud(p);

    