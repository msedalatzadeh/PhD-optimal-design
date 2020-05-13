clc
clear all
tic
%% Specify the parameters of the railway track model
EI=1
rhoa=1
k=1
l=1
alpha=0
delta=0.001/2
mu=0.1;
Cd=0.0001;
cw=1
cv=1
cu=1
T=60
mesh=600
tspan=[0:0.05:15];
linspace(0,T,mesh);
u1=0.01*sin(tspan*pi/T);
r1=0.1*l;

hc_comp=figure
%% Initial conditions
N=2
z0=[1,-1]'
par={[EI,rhoa,k,l,alpha,delta,mu,Cd,cw,cv,cu],tspan,z0}
%%
options=optimoptions('fmincon','Display','off','SpecifyObjectiveGradient'...
    ,true,'ScaleProblem',true,'MaxIterations',3,'UseParallel',true...
    ,'Algorithm','interior-point','SubproblemAlgorithm'...
    ,'cg')
rmin=0+par{1}(6);
rmax=par{1}(4)-par{1}(6);
lb=[-10*ones(1,length(tspan)),rmin];
ub=[+10*ones(1,length(tspan)),rmax];
[ur,Cost]=fmincon(@(ur)RTcostgrad(ur,par),[u1,r1],[],[],[],[],lb,ub,[]...
    ,options);
m=length(tspan);
u_fmin=ur(1:m);
disp(Cost)
r_fmin=ur(m+1);

n=length(z0);
m=length(tspan);
Bn=str2func(sprintf('B%d',n));
A=An(n,par{1});
Q=Qn(n,par{1});
J_lqr=zeros(1,3);
parfor i=1:3
    r(i)=0+(i-1)/2*l;
    B=Bn(r(i),par{1});
    [K,S,e]=lqr(A,B,Q,cu);
    J_lqr(i)=z0'*S*z0;
    %z_lqr(:,i)=expm((A-B*K)*tspan(i))*z0;
    %u_lqr(i)=-K*z_lqr(:,i);
end

J_fmin=zeros(1,3);
parfor i=1:3
    r(i)=0+(i-1)/2*l
    %J_fmin(i)=cost(u_fmin,r(i),par);
    x=r(i);
    [~,J_fmin(i)]=fmincon(@(u)RTcostgrad_u(u,x,par),u_fmin,[],[],[],[]...
        ,lb(1:m),ub(1:m),[],options);
    %J_fmin(i)=y;
end

%save('workspace3.mat')
plot(r,J_fmin,r,J_lqr)
legend('Nonlinear model','Linear model')
xlabel('Actuator location, $r$')
ylabel('Cost, $J(u^o,r;\textbf{z}_0)$')
toc