% minimum snap trajectory generation 
% ������Ʈ 66p 


% parameters: 
% m= # of key frame(set point) => ts=[t0 t1 t2 ... tm]
% n= order of polynomials
% parameter =
% [p1 p2 p3 p4 ... pn] pi=[pi1 pi2 pi3 pi4 ... pim] 
% pij=[rij kji]  


%% objective function 
global m n ts
n=8; ts=[0 1 2 3 4];
m=length(ts)-1;
C=zeros(n,n,m);


M=zeros(4*(n+1)*m);
keyframe=zeros(4,5); % [r_T yaw_T]'*(# of keyframe)

% as an example, keyframes are incremetally ...
for i=1:5
    keyframe(:,i)=(i-1)*ones(4,1);
end

% position objective

for k=1:m
    for i=4:n
        for j=4:n
            if i==4 && j==4
            C(i,j,k)=i*(i-1)*(i-2)*(i-3)*j*(j-1)*(j-2)*(j-3)*(ts(k+1)-ts(k));
            else
            C(i,j,k)=i*(i-1)*(i-2)*(i-3)*j*(j-1)*(j-2)*(j-3)*1/(i+j-8)*(ts(k+1)^(i+j-7)-ts(k)^(i+j-7));
            end
        end
    end
end


for i=1:n
    for j=1:n
        Mij=zeros(4*m);
        for k=1:m
            Mij(4*k-3:4*k,4*k-3:4*k)=C(i,j,k)*blkdiag(eye(3),0);
        end
        M(4*m*(i)+1:4*m*(i+1),4*m*(j)+1:4*m*(j+1))=Mij;    
    end
end

Mr=M;
%%
% yaw objective
C=zeros(n,n,m);

for k=1:m
    for i=2:n
        for j=2:n
            if i==2 && j==2
            C(i,j,k)=i*(i-1)*j*(j-1)*(ts(k+1)-ts(k));
            else
            C(i,j,k)=i*(i-1)*j*(j-1)*1/(i+j-4)*(ts(k+1)^(i+j-3)-ts(k)^(i+j-3));
            end
        end
    end
end


for i=1:n
    for j=1:n
        Mij=zeros(4*m);
        for k=1:m
            Mij(4*k-3:4*k,4*k-3:4*k)=C(i,j,k)*blkdiag(0,eye(3));
        end
        M(4*m*(i)+1:4*m*(i+1),4*m*(j)+1:4*m*(j+1))=Mij;    

    end
end

M_ksi=M;




%% constraint matrix for continuity 
T=[];

for i=1:n+1 % actually from 0 to n
    Ti=zeros(4*m);
    for k=1:m % actually from 0 to m
        Ti(4*(k-1)+1:4*(k),4*(k-1)+1:4*(k))=ts(k)^(i-1)*eye(4);
    end
    k=m+1;
        Ti(4*(k-1)+1:4*(k),4*(k-2)+1:4*(k-1))=ts(k)^(i-1)*eye(4);
    T=[T Ti];
end

%% optimization 
keyframe=reshape(keyframe,numel(keyframe),1);
Mobj=Mr+M_ksi;
bnd_cond=quadprog(Mobj,[],[],[],T,keyframe);


%% plot the trajectory 


figure
plot3(keyframe(1,:),keyframe(2,:),keyframe(3,:),'r*')
hold on 
axis([0 10 0 10 0 10])

%% new trial 
% refet p71-P73
n=6; % order of polynomial

C=100*rand*ones(3*(n+1));
C=zeros(3*(n+1));
t0=0; tf=10; tm=5;


for i=4:8
    for j=4:8
        if i==4 && j==4
        C(3*i+1:3*(i+1),3*j+1:3*(j+1))=(factorial(i)/factorial(i-4)*factorial(j)/factorial(j-4)*(tf-t0))*eye(3);
        else
        C(3*i+1:3*(i+1),3*j+1:3*(j+1))=(factorial(i)/factorial(i-4)*factorial(j)/factorial(j-4)/(i+j-8)*(tf^(i+j-7)-t0^(i+j-7)))*eye(3);
        end
    end
end

% POSTION CONTRAINT 
% ===============
x0=ones(3,1); xf=[10 5 -2]';
Tx=repmat(eye(3),2,1);
for i=1:n
    Tx=[Tx [t0^i*eye(3) ; tf^i*(eye(3)) ]];    
end

% VELOCITY CONSTRAINT
% ================
dx0=zeros(3,1); dxf=xf-x0;
Tdx=repmat(zeros(3),2,1);
for i=1:n
    Tdx=[Tdx [i*t0^(i-1)*eye(3) ; i*tf^(i-1)*(eye(3)) ]];    
end

% ACCELERATION CONSTRAINT (direction only)
% ====================
ad=[1 1 1]';
Td2x=repmat(zeros(3),1,2);
for i=2:n
    Td2x=[Td2x (i-1)*i*tm^(i-2)*eye(3)];
end
aC=[ad(2) -ad(1) 0;0 ad(3) -ad(2)]*Td2x;

% CORRIDER CONSTRAINT
% ================




% OPIMIZATION
% =========

T=[Tx;Tdx;aC];
bnd_cond=[x0; xf; dx0; dxf;[0 0]'];

p=quadprog(C,[],[],[],T,bnd_cond);

 PLOTTING
x=[]; d2x=[]; t=[];
for time=0:0.05:10
    t=[t  time];
    traj_res=traj(p,time);
    x=[x traj_res.x];
    d2x=[d2x traj_res.d2x];
end

plot3(x(1,:), x(2,:), x(3,:))
hold on 
quiver3(x(1,101),x(2,101),x(3,101),d2x(1,101),d2x(2,101),d2x(3,101))








        