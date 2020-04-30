clc
clear all
tic
%% Specify the parameters of the railway track model
z0=[1.5,1.4,1.3,1.2,1.1,1,0.9,0.8,0.7,0.6]'
EI=1
rhoa=1
k=0
l=1
alpha=0
delta=0.1/2
mu=0.1
Cd=0.001
cw=1
cv=1
cu=10
T=25
mesh=350
tspan=linspace(0,T,mesh)
par={[EI,rhoa,k,l,alpha,delta,mu,Cd,cw,cv,cu],tspan,z0}
%% Initialize input and actuator location
u1=0.01*sin(tspan*pi/T);
norm(z0,2)*rand(1,mesh)/10;
r1=0.9*l;
%%
[un,rn,zn,pn,DuJn,DrJn]=NCGM(u1,r1,par,'Wolfe','FR',0.01);
%[un,rn,zn,pn,DuJ,DrJ]=PGM(u1,r1,par,'strongWolfe',0.001);


n=length(un)
uo=un{n};
ro=rn{n}
zo=zn{n};
po=pn{n};
Jo=cost(uo,ro,par)

%%
parfor i=1:101
   r(i)=0+(i-1)/100*l;
   J(i)=cost(uo,r(i),par);
end
%%
n=length(z0);
m=length(tspan);
wmo=zeros(1,m);
for i=1:m
   for j=1:(n/2)
   wmo(i)=wmo(i)+(zo(2*j-1,i)-zo(2*j,i))*c(j,par{1})*1/(j^2*pi^2)*sin(j*pi/2);
   end
end

vmo=zeros(1,m);
for i=1:m
   for j=1:(n/2)
   vmo(i)=vmo(i)+(po(2*j-1,i)-po(2*j,i))*c(j,par{1})*1/(j^2*pi^2)*sin(j*pi/2);
   end
end
%%
run('MatlabOptimizationToolbox')
%%
Bn=str2func(sprintf('B%d',n));
A=An(n,par{1});
B=Bn(r_fmin,par{1});
Q=Qn(n,par{1});
[K,S,e]=lqr(A,B,Q,cu);
J_lqr=z0'*S*z0
for i=1:m
z_lqr(:,i)=expm((A-B*K)*tspan(i))*z0;
u_lqr(i)=-K*z_lqr(:,i);
end
%p_lqr=lsim(ss(-A',Q,[],[]),ones(m,n),fliplr(tspan),0*z0)
wmo_lqr=zeros(1,m);
for i=1:m
   for j=1:(n/2)
   wmo_lqr(i)=wmo_lqr(i)+(z_lqr(2*j-1,i)-z_lqr(2*j,i))*c(j,par{1})*1/(j^2*pi^2)*sin(j*pi/2);
   end
end

% vmo_lqr=zeros(1,m);
% for i=1:m
%    for j=1:(n/2)
%    vmo_lqr(i)=vmo_lqr(i)+(p_lqr(2*j-1,i)-p_lqr(2*j,i))*c(j,par{1})*1/(j^2*pi^2)*sin(j*pi/2);
%    end
% end
%%
for i=1:101
normB(i)=norm(Bn(r(i),par{1}),2);
end
figure
plot(r,normB)
%%
run('Plotter')
%%
toc
