% function y=cost(u,r,par)
% tspan=par{2};
% z0=par{3};
% n=length(z0);
% Q=Qn(n,par{1});
% R=par{1}(11);
% [z,~,~,~]=RTsolver(u,r,par);
% m=length(tspan);
% J=zeros(1,m);
% for i=1:m
% J(i)=z(:,i)'*Q*z(:,i)+u(:,i)'*R*u(:,i);
% end
% y=trapz(tspan,J);
% end

function J=cost(u,r,par)
tspan=par{2};
z0=par{3};
n=length(z0);
A=An(n,par{1});
Q=Qn(n,par{1});
R=par{1}(11);
F=str2func(sprintf('F%d',n));
B=str2func(sprintf('B%d',n));
F=@(z)F(z,par{1});
B=B(r,par{1});

options=odeset('RelTol',1e-5,'AbsTol',1e-6);
sol_IVP=ode15s(@(t,z)IVP(t,z,interp1(tspan,u,t),A,F(z),B),tspan,z0,options);
z=deval(sol_IVP,tspan);
m=length(tspan);
L=zeros(1,m);
for i=1:m
L(i)=z(:,i)'*Q*z(:,i)+u(:,i)'*R*u(:,i);
end
J=trapz(tspan,L);
end