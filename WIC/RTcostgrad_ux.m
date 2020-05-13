function [Cost,Gradient]=RTcostgrad_ux(ux,r,par)
%par={[EI rhoa k l alpha delta mu Cd cw cv cu],tspan}
tspan=par{2};
n=par{3};
m=length(tspan);
u=ux(1:m);
x0=ux(m+1:m+n)';

A=An(n,par{1});
Q=Qn(n,par{1});
R=par{1}(11);

F=str2func(sprintf('F%d',n));
B=str2func(sprintf('B%d',n));
dF=str2func(sprintf('dF%d',n));
F=@(x)F(x,par{1});
B=@(r)B(r,par{1});
dF=@(x)dF(x,par{1});

options=odeset('RelTol',1e-5,'AbsTol',1e-6);
sol_IVP=ode15s(@(t,x)IVP(t,x,interp1(tspan,u,t),A,F(x),B(r)),tspan,x0,options);
sol_FVP=ode15s(@(t,p)FVP(t,p,deval(sol_IVP,t),A,dF(deval(sol_IVP,t)),Q),fliplr(tspan),0*x0,options);
x=deval(sol_IVP,tspan);
p=deval(sol_FVP,tspan);

DuJ=2*(B(r)'*p+R*u);
DxJ=-2*p(:,1)';
Gradient=[DuJ,DxJ];

m=length(tspan);
L=zeros(1,m);
for i=1:m
L(i)=x(:,i)'*Q*x(:,i)+u(:,i)'*R*u(:,i);
end
Cost=trapz(tspan,L);
end