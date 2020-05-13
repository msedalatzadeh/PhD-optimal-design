T=10
mesh=1000
tspan=linspace(0,T,mesh);
u1=1*cos(0.1*tspan*pi/T);
r1=0.5*l;
z0=[2,3]'
par={[EI,rhoa,k,l,Alpha,delta,mu,Cd,cw,cv,cu],tspan,z0}
cost(u1,r1,par)
[z,p,DuJ,DrJ]=RTsolver(u1,r1,par)
plot(tspan,DuJ)
hold on
plot(tspan,p(1,:))

save('ToPython.mat')
[Costt,grad]=RTcostgrad([u_fmin,r_fmin],par);
plot(tspan,grad(1:mesh))