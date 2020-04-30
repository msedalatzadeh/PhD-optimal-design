function [Cost,Gradient]=RTcostgrad_u(u,r,par)
%par={[EI rhoa k l alpha delta mu Cd cw cv cu],tspan,z0}
tspan=par{2};
z0=par{3};

n=length(z0);
A=An(n,par{1});
Q=Qn(n,par{1});
R=par{1}(11);

F=str2func(sprintf('F%d',n));
B=str2func(sprintf('B%d',n));
dF=str2func(sprintf('dF%d',n));
F=@(z)F(z,par{1});
B=@(r)B(r,par{1});
dF=@(z)dF(z,par{1});

options=odeset('RelTol',1e-5,'AbsTol',1e-6);
sol_IVP=ode15s(@(t,z)IVP(t,z,interp1(tspan,u,t),A,F(z),B(r)),tspan,z0,options);
sol_FVP=ode15s(@(t,p)FVP(t,p,deval(sol_IVP,t),A,dF(deval(sol_IVP,t)),Q),fliplr(tspan),0*z0,options);
z=deval(sol_IVP,tspan);
p=deval(sol_FVP,tspan);

DuJ=2*(B(r)'*p+R*u);
Gradient=DuJ;

m=length(tspan);
L=zeros(1,m);
for i=1:m
L(i)=z(:,i)'*Q*z(:,i)+u(:,i)'*R*u(:,i);
end
Cost=trapz(tspan,L);
end