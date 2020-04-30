clc
clear all
%% Parameters
N=4
mu_exp=-1
Cd_exp=-4
%% Constants
EI=1
rhoa=1
k=1
l=1
Alpha=10
delta=0.001/2
mu=10^mu_exp;
Cd=10^Cd_exp;
cw=1
cv=1
cu=1
T=60
mesh=600
tspan=linspace(0,10,2500);
linspace(0,T,mesh);

u1=0.01*sin(tspan*pi/T);
r1=0.1*l;

z0=[2,3,2/2,3/2,2/4,3/4,2/8,3/8,2/16,3/16]'
z0=z0(1:N)
par={[EI,rhoa,k,l,Alpha,delta,mu,Cd,cw,cv,cu],tspan,z0}
%% Optimizer
options=optimoptions('fmincon','Display','iter','SpecifyObjectiveGradient'...
   ,false,'ScaleProblem',true ...
   ,'Algorithm','interior-point','SubproblemAlgorithm','cg')
rmin=0+par{1}(6);
rmax=par{1}(4)-par{1}(6);
lb=[-10*ones(1,length(tspan)),rmin];
ub=[+10*ones(1,length(tspan)),rmax];

tic
[ur,Cost]=fmincon(@(ur)RTcostgrad(ur,par),[u1,r1],[],[],[],[],lb,ub,[]...
   ,options);
cputime=toc
m=length(tspan);
u_fmin=ur(1:m);
r_fmin=ur(m+1);