function [x,p,DuJ,DrJ,DxJ,Cost]=RTsolver(urx,par)
%par={[EI rhoa k l alpha delta mu Cd cw cv cu],tspan,n}
tspan=par{2};
n=par{3};
m=length(tspan);
u=urx(1:m);
r=urx(m+1);
x0=urx(m+2:m+1+n)';

A=An(n,par{1});
Q=Qn(n,par{1});
R=par{1}(11);

F=str2func(sprintf('F%d',n));
B=str2func(sprintf('B%d',n));
dF=str2func(sprintf('dF%d',n));
dB=str2func(sprintf('dB%d',n));
F=@(x)F(x,par{1});
B=@(r)B(r,par{1});
dF=@(x)dF(x,par{1});
dB=@(r)dB(r,par{1});

options=odeset('RelTol',1e-5,'AbsTol',1e-6);
sol_IVP=ode15s(@(t,x)IVP(t,x,interp1(tspan,u,t),A,F(x),B(r)),tspan,x0,options);
sol_FVP=ode15s(@(t,p)FVP(t,p,deval(sol_IVP,t),A,dF(deval(sol_IVP,t)),Q),...
   fliplr(tspan),zeros(n,1),options);
x=deval(sol_IVP,tspan);
p=deval(sol_FVP,tspan);

DuJ=2*(B(r)'*p+R*u);


DrJ=zeros(1,m);
for i=1:m
DrJ(i)=2*u(:,i)'*dB(r)*p(:,i);
end
DrJ=trapz(tspan,DrJ);


L=zeros(1,m);
for i=1:m
L(i)=x(:,i)'*Q*x(:,i)+u(:,i)'*R*u(:,i);
end
DxJ=-2*p(:,1)';
Cost=trapz(tspan,L);
end

