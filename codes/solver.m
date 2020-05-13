function [z,p,DuJ,DrJ]=solver(u,r,z0,tspan,An,Fn,Bn,dFn,dBn,Qn,Rn)

options=odeset('RelTol',1e-5,'AbsTol',1e-6);
sol_IVP=ode15s(@(t,z)IVP(t,z,interp1(tspan,u,t),An,Fn(z),Bn(r)),tspan,z0,options);
sol_FVP=ode15s(@(t,p)FVP(t,p,deval(sol_IVP,t),An,dFn(deval(sol_IVP,t)),Qn),fliplr(tspan),0*z0,options);
z=deval(sol_IVP,tspan);
p=deval(sol_FVP,tspan);

DuJ=2*(Bn(r)'*p+Rn*u);

m=length(tspan);
DrJ=zeros(1,m);
for i=1:m
DrJ(i)=2*u(:,i)'*dBn(r)*p(:,i);
end
DrJ=trapz(tspan,DrJ);

end

