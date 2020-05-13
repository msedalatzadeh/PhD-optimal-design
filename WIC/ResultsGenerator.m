clc
clear all
%% Parameters
N=2
mu_exp=-1		
Cd_exp=-4
Alpha=10
mesh=3500
cu=1
x1=[2,3,2/2,3/2,2/4,3/4,2/8,3/8,2/16,3/16,2/32,3/32]/18
T=10
p=parpool(15);
p.IdleTimeout=Inf;
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
tspan=linspace(0,T,mesh);

u1=sin(tspan*pi/T);
r1=0.9*l;
x1=x1(1:N)
n=length(x1)
par={[EI,rhoa,k,l,Alpha,delta,mu,Cd,cw,cv,cu],tspan,n}
%% Optimizer uo and ro
disp('Optimizer uo and ro')
options=optimoptions('fmincon','Display','off','SpecifyConstraintGradient',...
    true,'SpecifyObjectiveGradient',true,'HonorBounds',true,...
    'Algorithm',...
   'interior-point','SubproblemAlgorithm','cg','HessianApproximation','bfgs');
rmin=0+par{1}(6);
rmax=par{1}(4)-par{1}(6);
lb=[-10*ones(1,length(tspan)),rmin,-Inf*x1];
ub=[+10*ones(1,length(tspan)),rmax,+Inf*x1];

tic
m=length(tspan);
[urx,Cost]=fmincon(@(urx)RTcostgrad(urx,par),[u1,r1,x1],[],[],[],[],...
           lb,ub,@(urx)Constraint(urx,m,n),options);
cputime=toc;
u_fmin=urx(1:m);
r_fmin=urx(m+1);
x_fmin=urx(m+2:m+1+n);
%% Cost in nonlinear system vs r
disp('Cost in nonlinear system vs r')
J_fmin=zeros(1,200);
r=linspace(0,1,200);
lb=[-10*ones(1,length(tspan)),-Inf*x1];
ub=[+10*ones(1,length(tspan)),+Inf*x1];
parfor i=1:200
   rr=r(i);
   [~,J_fmin(i)]=fmincon(@(ux)RTcostgrad_ux(ux,rr,par),[u_fmin,x_fmin],[],[],[],[]...
      ,lb,ub,@(ux)Constraint_ux(ux,m,n),options);
end
%% Initial Condition
disp('Initial Condition')
mesh=100;
x=linspace(0,l,mesh);
i=1;
w0=zeros(1,mesh);
v0=zeros(1,mesh);
while i<=mesh
    for n=1:N/2
        w0(i)=w0(i)+x_fmin(2*n-1)*c(n,par{1})*sin(n*pi*x(i)/l)/(n^2*pi^2)...
            -x_fmin(2*n)*c(n,par{1})*sin(n*pi*x(i)/l)/(n^2*pi^2);
        v0(i)=v0(i)+x_fmin(2*n-1)*c(n,par{1})*sin(n*pi*x(i)/l)...
            -x_fmin(2*n)*c(n,par{1})*sin(n*pi*x(i)/l);
    end
    i=i+1;
end
%%
save(sprintf('WIC-%gmode&Alpha%g&Cd%g&mu%giterInfcu%g.mat',N,Alpha,Cd_exp,mu_exp,cu))
