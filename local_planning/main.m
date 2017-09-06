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

%% new trial % refet p71-P73
n=8; % order of polynomial
% for better condition number of C matrix 
C=10*rand(3*(n+1),1);

% IN REAL TIME DOMAIN 
%%%%%%%%%%%%%%

t0_real=1; tf_real=10; tm_real=[9 10]; t_cond_real=[t0_real tf_real tm_real];

% BOUNDARY CONDITION
%%%%%%%%%%%%%%%
x0=zeros(3,1); xf=10*ones(3,1); 
dx0_real=ones(3,1)/5;



% in non-dimensionalized domain:
% mapping btw two domain
t_cond=(t_cond_real-t0_real)/(tf_real-t0_real);
t0=t_cond(1); tf=t_cond(2); tm=t_cond(3:end);

for i=4:n
    for j=4:n
        if i==4 && j==4
        C(3*i+1:3*(i+1),3*j+1:3*(j+1))=(factorial(i)/factorial(i-4)*factorial(j)/factorial(j-4)*(tf-t0))*eye(3);
        else
        C(3*i+1:3*(i+1),3*j+1:3*(j+1))=(factorial(i)/factorial(i-4)*factorial(j)/factorial(j-4)/(i+j-8)*(tf^(i+j-7)-t0^(i+j-7)))*eye(3);
        end
    end
end

Aeq=[]; beq=[];
A=[]; b=[];

% POSTION CONTRAINT 
% ===============
Aeq=[Aeq ;[Tmat(t0,0,n) ; Tmat(tf,0,n)]];
beq=[beq ;x0;xf];
% VELOCITY CONSTRAINT
% ================
dx0=dx0_real*(tf-t0);
Aeq=[Aeq ; Tmat(t0,1,n)];
beq=[beq;dx0];

%ACCELERATION CONSTRAINT (direction only)
% ====================
ad=[0.1 0.5 0.5]';

for i=1:length(tm)
% (1) equality condition

Aeq=[Aeq ; [ad(2) -ad(1) 0;0 ad(3) -ad(2)]*Tmat(tm(i),2,n)];
beq=[beq; zeros(2,1)];

% (2) inequailty condition = same direction (very improtant)
A=[A ;[-ad(1) 0 0;0 0 0; 0 0 0]*Tmat(tm(i),2,n)]; 
b=[b; zeros(3,1)];

end

% CORRIDER CONSTRAINT
% ================
n_seg=5; tlist=linspace(t0,tf,n_seg); delta=1.5;
t=(xf-x0)/norm(xf-x0);
r=x0;
for i=2:n_seg-1
   T=Tmat(tlist(i),0,n);
   for j=1:3
       A=[A;(T(j,:)-t(j)*t'*T)]; b=[b;delta-(t'*r)*t(j)-r(j)];
       A=[A;-(T(j,:)-t(j)*t'*T)]; b=[b;delta+(t'*r)*t(j)-r(j)];
   end    
end
    

% OPIMIZATION
% =========
% options = optimoptions('quadprog','Display','off',);

p=quadprog(C,[],A,b,Aeq,beq);
%p=fmincon(@(x) x'*C*x,rand(3*(n+1),1),A,b,Aeq,beq);


%% PLOTTING
x=[]; d2x=[]; tset=[];
for time=t0_real:tf_real
    tset=[tset  time];
    traj_res=traj(p,time,n,t0_real,tf_real);
    x=[x traj_res.x];
    d2x=[d2x traj_res.d2x];
end


figure

% tmpAspect=daspect();
% daspect(tmpAspect([1 2 2]))
hold on 

plot3(x0(1),x0(2),x0(3),'bo')
plot3(xf(1),xf(2),xf(3),'ro')
plot3(x(1,:), x(2,:), x(3,:))


for time=linspace(t0,tf,20)

traj_res=traj(p,time,n,t0,tf);

quiver3(traj_res.x(1),traj_res.x(2),traj_res.x(3),traj_res.d2x(1)/norm(traj_res.d2x),traj_res.d2x(2)/norm(traj_res.d2x),traj_res.d2x(3)/norm(traj_res.d2x),'Color','r','LineWidth',1,'MaxHeadSize',2);
%quiver3(traj_res.x(1),traj_res.x(2),traj_res.x(3),traj_res.d2x(1),traj_res.d2x(2),traj_res.d2x(3),'Color','r','LineWidth',1,'MaxHeadSize',2);
end

figure

for i=1:3
    subplot(3,1,i)
    plot(tset,x(i,:))
end


