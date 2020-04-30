clc
clear all
%% Parameters
N=6
mu_exp=-1
Cd_exp=-4
Alpha=10
mesh=2500
z0=[2,3,2/2,3/2,2/4,3/4,2/8,3/8,2/16,3/16]'
%parpool(19)
%% Constants
EI=1    
rhoa=1
k=1
l=1
delta=0.001/2
mu=10^mu_exp;
Cd=10^Cd_exp;
cw=1
cv=1
cu=1
T=10
tspan=linspace(0,T,mesh);

u1=sin(tspan*pi/T);
r1=0.9*l;
z0=z0(1:N)
par={[EI,rhoa,k,l,Alpha,delta,mu,Cd,cw,cv,cu],tspan,z0}
%% Optimizer uo and ro
disp('Optimizer uo and ro')
options=optimoptions('fmincon','Display','off','SpecifyObjectiveGradient'...
   ,true,'HonorBounds',true,'MaxIterations',35,'Algorithm',...
   'interior-point','SubproblemAlgorithm','cg','HessianApproximation','bfgs');
rmin=0+par{1}(6);
rmax=par{1}(4)-par{1}(6);
lb=[-10*ones(1,length(tspan)),rmin];
ub=[+10*ones(1,length(tspan)),rmax];

tic
[ur,Cost]=fmincon(@(ur)RTcostgrad(ur,par),[u1,r1],[],[],[],[],lb,ub,[]...
   ,options);
cputime=toc;
m=length(tspan);
u_fmin=ur(1:m);
r_fmin=ur(m+1);
%% Cost in linear system vs r
disp('Cost in linear system vs r')
n=length(z0);
m=length(tspan);
Bn=str2func(sprintf('B%d',n));
A=An(n,par{1});
Q=Qn(n,par{1});
J_lqr=zeros(1,200);
r=linspace(delta,1-delta,200);
parfor i=1:200
   B=Bn(r(i),par{1});
   [~,S,~]=lqr(A,B,Q,cu);
   J_lqr(i)=z0'*S*z0;
end
%% Beam vibrations
disp('Beam vibrations')
B=Bn(r_fmin,par{1});
[K,~,~]=lqr(A,B,Q,cu);
parfor i=1:m
   z_lqr(:,i)=expm((A-B*K)*tspan(i))*z0;
   u_lqr(i)=-K*z_lqr(:,i);
end
w_lqr=zeros(1,m);
parfor i=1:m
   for j=1:(n/2)
      w_lqr(i)=w_lqr(i)+(z_lqr(2*j-1,i)-z_lqr(2*j,i))*c(j,par{1})*1/(j^2*pi^2)*sin(j*pi/2);
   end
end

[z_fmin,~,~,~,~]=RTsolver([u_fmin,r_fmin],par);
w_fmin=zeros(1,m);
parfor i=1:m
   for j=1:(n/2)
      w_fmin(i)=w_fmin(i)+(z_fmin(2*j-1,i)-z_fmin(2*j,i))*c(j,par{1})*1/(j^2*pi^2)*sin(j*pi/2);
   end
end
%% Cost in nonlinear system vs r
disp('Cost in nonlinear system vs r')
J_fmin=zeros(1,200);
parfor i=1:200
   x=r(i);
   [~,J_fmin(i)]=fmincon(@(u)RTcostgrad_u(u,x,par),u_fmin,[],[],[],[]...
      ,lb(1:m),ub(1:m),[],options);
end
%% Mid-point cost and input
disp('Mid-point cost and input')
[u_mid,J_mid]=fmincon(@(u)RTcostgrad_u(u,l/2,par),u_fmin,[],[],[],[]...
   ,lb(1:m),ub(1:m),[],options);
%% Initial Condition
disp('Initial Condition')
mesh=100;
x=linspace(0,l,mesh);
i=1;
w0=zeros(1,mesh);
v0=zeros(1,mesh);
while i<=mesh
    for n=1:N/2
        w0(i)=w0(i)+z0(2*n-1)*c(n,par{1})*sin(n*pi*x(i)/l)/(n^2*pi^2)...
            -z0(2*n)*c(n,par{1})*sin(n*pi*x(i)/l)/(n^2*pi^2);
        v0(i)=v0(i)+z0(2*n-1)*c(n,par{1})*sin(n*pi*x(i)/l)...
            -z0(2*n)*c(n,par{1})*sin(n*pi*x(i)/l);
    end
    i=i+1;
end
%% Changing alpha
% disp('Changing alpha')
% m=length(tspan);
% parfor i=1:101
%    alpha(i)=(i-1)/2.5;
%    par={[EI,rhoa,k,l,alpha(i),delta,mu,Cd,cw,cv,cu],tspan,z0};
% [ur,Cost_alpha(i)]=fmincon(@(ur)RTcostgrad(ur,par),[u1,r1],[],[],[],[],lb,ub,[]...
%    ,options);
% u_alpha{i}=ur(1:m);
% r_alpha(i)=ur(m+1);
% end
%%
save(sprintf('%gmode&Alpha%g&Cd%g&mu%g.mat',N,Alpha,Cd_exp,mu_exp))
