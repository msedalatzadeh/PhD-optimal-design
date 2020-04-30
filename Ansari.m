%% Ansari paper
clc
clear all
tic
%% Specify the parameters of the railway track model
z0=[1,1,1,1]
EI=6415.5E3
rhoa=60.367
k=35E6
l=18
alpha=4E14
delta=l/10
mu=1732.5E3
Cd=0
cw=0
cv=0
cu=0
T=10
mesh=100
tspan=linspace(0,T,mesh)
par={[EI,rhoa,k,l,alpha,delta,mu,Cd,cw,cv,cu],tspan,z0}
%% Initialize input and actuator location
u1=0*65E3*sin(tspan);
r1=l/2;
%%
[z,p,DuJ,DrJ]=RTsolver(u1,r1,par)

for i=1:length(tspan)
   wmo(i)=0;
   for j=1:length(z0)
   wmo(i)=wmo(i)+z(j,i)*c(j,par{1})*1/(j^2*pi^2)*sin(j*pi/2);
   end
end