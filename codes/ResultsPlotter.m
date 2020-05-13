clc
clear all
%%
set(groot,'defaultLineLineWidth',1);
set(0,'defaultAxesFontSize',12);
set(groot,'defaulttextinterpreter','latex');
set(groot,'defaultAxesTickLabelInterpreter','latex');
set(groot,'defaultTextInterpreter','latex')
set(groot,'defaultLegendInterpreter','latex');
%% Increasing order of approximation
hincrease=figure('rend','painters','pos',[400 100 600 500])
load('2mode&Alpha10&Cd-4&mu-1')
subplot(2,1,1)
plot(r,J_fmin,'.');
hold on
Jo=min(J_fmin)
plot(l/2,Jo,'ko','HandleVisibility','off')

load('4mode&Alpha10&Cd-4&mu-1')
plot(r,J_fmin,':');
hold on
Jo=min(J_fmin)
ro=r(find(J_fmin==Jo))
ro=[ro,0.5+abs(0.5-ro)]
Jo=[Jo,Jo]
plot(ro,Jo,'ko','HandleVisibility','off')

load('6mode&Alpha10&Cd-4&mu-1')
plot(r,J_fmin,'-.');
hold on
Jo=min(J_fmin)
ro=r(find(J_fmin==Jo))
ro=[ro,0.5+abs(0.5-ro)]
Jo=[Jo,Jo]
plot(ro,Jo,'ko','HandleVisibility','off')

load('8mode&Alpha10&Cd-4&mu-1')
plot(r,J_fmin,'--');
hold on
Jo=min(J_fmin)
ro=r(find(J_fmin==Jo))
ro=[ro,0.5+abs(0.5-ro)]
Jo=[Jo,Jo]
plot(ro,Jo,'ko','HandleVisibility','off')

load('10mode&Alpha10&Cd-4&mu-1iterInf')
plot(r,J_fmin);
hold on
Jo=min(J_fmin)
ro=r(find(J_fmin==Jo))
ro=[ro,0.5+abs(0.5-ro)]
Jo=[Jo,Jo]
plot(ro,Jo,'ko')

legend('2nd order','4th order','6th order','8th order','10th order','optimizer','Location','North')

xlabel('$r$');
ylabel('$J(u^o(r),r;x_0)$');
title('(a) Cost');

load('2mode&Alpha10&Cd-4&mu-1')
subplot(2,1,2)
plot(tspan,u_fmin,'.');
hold on
load('4mode&Alpha10&Cd-4&mu-1')
plot(tspan,u_fmin,':');
load('6mode&Alpha10&Cd-4&mu-1')
plot(tspan,u_fmin,'-.');
load('8mode&Alpha10&Cd-4&mu-1')
plot(tspan,u_fmin,'--');
load('10mode&Alpha10&Cd-4&mu-1')
plot(tspan,u_fmin);
xlabel('$t$');
ylabel('$u^o(t;r^o)$');
axis tight
title('(b) Optimal input')
legend('2nd order','4th order','6th order','8th order','10th order')

saveas(hincrease,'increasing order','epsc')
%% Increasing order of approximation - undamped
hincrease=figure('rend','painters','pos',[400 100 600 500])
load('2mode&Alpha10&Cd-Inf&mu-InfiterInf')
subplot(2,1,1)
plot(r,J_fmin,'.');
hold on
Jo=min(J_fmin)
plot(l/2,Jo,'ko','HandleVisibility','off')

load('4mode&Alpha10&Cd-Inf&mu-InfiterInf')
plot(r,J_fmin,':');
hold on
Jo=min(J_fmin)
ro=r(find(J_fmin==Jo))
ro=[ro,0.5+abs(0.5-ro)]
Jo=[Jo,Jo]
plot(ro,Jo,'ko','HandleVisibility','off')

load('6mode&Alpha10&Cd-Inf&mu-InfiterInf')
plot(r,J_fmin,'-.');
hold on
Jo=min(J_fmin)
ro=r(find(J_fmin==Jo))
ro=[ro,0.5+abs(0.5-ro)]
Jo=[Jo,Jo]
plot(ro,Jo,'ko','HandleVisibility','off')

load('8mode&Alpha10&Cd-Inf&mu-InfiterInf')
plot(r,J_fmin,'--');
hold on
Jo=min(J_fmin)
ro=r(find(J_fmin==Jo))
ro=[ro,0.5+abs(0.5-ro)]
Jo=[Jo,Jo]
plot(ro,Jo,'ko','HandleVisibility','off')

load('10mode&Alpha10&Cd-Inf&mu-InfiterInf')
plot(r,J_fmin);
hold on
Jo=min(J_fmin)
ro=r(find(J_fmin==Jo))
ro=[ro,0.5+abs(0.5-ro)]
Jo=[Jo,Jo]
plot(ro,Jo,'ko')

legend('2nd order','4th order','6th order','8th order','10th order','optimizer','Location','North')

xlabel('$r$');
ylabel('$J(u^o(r),r;x_0)$');
title('(a) Cost');

load('2mode&Alpha1&Cd-Inf&mu-InfiterInf')
subplot(2,1,2)
plot(tspan,u_fmin,'.');
hold on
load('4mode&Alpha1&Cd-Inf&mu-InfiterInf')
plot(tspan,u_fmin,':');
load('6mode&Alpha1&Cd-Inf&mu-InfiterInf')
plot(tspan,u_fmin,'-.');
load('8mode&Alpha1&Cd-Inf&mu-InfiterInf')
plot(tspan,u_fmin,'--');
load('10mode&Alpha1&Cd-Inf&mu-InfiterInf')
plot(tspan,u_fmin);
xlabel('$t$');
ylabel('$u^o(t;r^o)$');
axis tight
title('(b) Optimal input')
legend('2nd order','4th order','6th order','8th order','10th order')
saveas(hincrease,'increasing order -undamped','epsc')
%% Comparison: nonlinear vs linear -damped
hcomp1=figure
subplot(2,1,2)
load('10mode&Alpha100&Cd-4&mu-1iterInf')
plot(tspan,u_fmin,'b')
load('10mode&Alpha0&Cd-4&mu-1')
hold on
plot(tspan,u_fmin,'--r')
legend('$\alpha=100$','$\alpha=0$')
xlabel('$t$')
ylabel('$u^o(t;r^o)$')
axis tight
title('(a) Optimal input')

subplot(2,1,1)
load('10mode&Alpha100&Cd-4&mu-1iterInf')
plot(r,J_fmin,'b')
hold on
Jo=min(J_fmin)
ro=r(find(J_fmin==Jo))
ro=[ro,0.5+abs(0.5-ro)]
Jo=[Jo,Jo]
plot(ro,Jo,'ko','HandleVisibility','off')
load('10mode&Alpha0&Cd-4&mu-1')
plot(r,J_fmin,'--r')
Jo=min(J_fmin)
ro=r(find(J_fmin==Jo))
ro=[ro,0.5+abs(0.5-ro)]
Jo=[Jo,Jo]
plot(ro,Jo,'ko')
xlabel('$r$')
ylabel('$J(u^o(r),r;x_0)$')
title('(b) Cost')
legend('$\alpha=100$','$\alpha=0$','optimizer','Location','North')

print('linearVSnonlinear-damping','-depsc')
%% Comparison: nonlinear vs linear -undamped
hcomp2=figure
subplot(2,1,1)
load('10mode&Alpha100&Cd-Inf&mu-InfiterInf')
plot(tspan,u_fmin,'b')
load('10mode&Alpha0&Cd-Inf&mu-InfiterInf')
hold on
plot(tspan,u_fmin,'--r')
xlabel('$t$')
ylabel('$u^o(t;r^o)$')
axis tight
title('(a) Optimal input')
legend('$\alpha=100$','$\alpha=0$')


subplot(2,1,2)
load('10mode&Alpha100&Cd-Inf&mu-InfiterInf')
plot(r,J_fmin,'b')
hold on
Jo=min(J_fmin)
ro=r(find(J_fmin==Jo))
ro=[ro,0.5+abs(0.5-ro)]
Jo=[Jo,Jo]
plot(ro,Jo,'ko','HandleVisibility','off')
load('10mode&Alpha0&Cd-Inf&mu-InfiterInf')
plot(r,J_fmin,'--r')
Jo=min(J_fmin)
ro=r(find(J_fmin==Jo))
ro=[ro,0.5+abs(0.5-ro)]
Jo=[Jo,Jo]
plot(ro,Jo,'ko')
xlabel('$r$')
ylabel('$J(u^o(r),r;x_0)$')
title('(b) Cost')
legend('$\alpha=100$','$\alpha=0$','optimizer','Location','North')

print('linearVSnonlinear-nodamping','-depsc')
%% Comparison: optimal location vs mid-point
hmid=figure
subplot(2,1,2)
load('10mode&Alpha10&Cd-4&mu-1iterInfcu100.mat')
plot(tspan,u_fmin,'b',tspan,u_mid,'--r')
xlabel('$t$')
ylabel('$u^o(t)$')
title('(a) Input')
axis tight
legend(sprintf('at $r^o$=%1.2f',r_fmin),sprintf('at $r$=%g',l/2))

subplot(2,1,1)
load('10mode&Alpha10&Cd-4&mu-1iterInfcu100.mat')
plot(tspan,w_fmin,'b',tspan,w_mid,'--r')
xlabel('$t$')
ylabel('$w(\frac{\ell}{2},t)$')
axis tight
title('(b) Deflection')
legend(sprintf('at $r^o$=%1.2f',r_fmin),sprintf('at $r$=%g',l/2))

print('optimalVSmidpoint','-depsc')
%% Initial condition
hini=figure
load('10mode&Alpha10&Cd-4&mu-1iterInf.mat')
plot(x,w0,x,v0)
legend('Initial deflection, $w_0(\xi)$','Initial velocity, $v_0(\xi)$',...
   'Location','SouthEast')
title('Initial condition')
xlabel('$\xi$')
ylabel('$w_0(\xi)$, and $v_0(\xi)$')
print('initial condition','-depsc')
%% Effect of alpha
halpha=figure
load('10mode&Alpha0&Cd-Inf&mu-InfiterInf.mat')
plot(r,J_fmin)
hold on
Jo=min(J_fmin)
ro=r(find(J_fmin==Jo))
ro=[ro,0.5+abs(0.5-ro)]
Jo=[Jo,Jo]
plot(ro,Jo,'ko','HandleVisibility','off')
load('10mode&Alpha1&Cd-Inf&mu-InfiterInf.mat')
plot(r,J_fmin,'.')
Jo=min(J_fmin)
ro=r(find(J_fmin==Jo))
ro=[ro,0.5+abs(0.5-ro)]
Jo=[Jo,Jo]
plot(ro,Jo,'ko','HandleVisibility','off')
load('10mode&Alpha10&Cd-Inf&mu-InfiterInf.mat')
plot(r,J_fmin,'--')
Jo=min(J_fmin)
ro=r(find(J_fmin==Jo))
ro=[ro,0.5+abs(0.5-ro)]
Jo=[Jo,Jo]
plot(ro,Jo,'ko','HandleVisibility','off')
load('10mode&Alpha50&Cd-Inf&mu-InfiterInf.mat')
plot(r,J_fmin,'-.')
Jo=min(J_fmin)
ro=r(find(J_fmin==Jo))
ro=[ro,0.5+abs(0.5-ro)]
Jo=[Jo,Jo]
plot(ro,Jo,'ko','HandleVisibility','off')
load('10mode&Alpha100&Cd-Inf&mu-InfiterInf.mat')
plot(r,J_fmin,':')
Jo=min(J_fmin)
ro=r(find(J_fmin==Jo))
ro=[ro,0.5+abs(0.5-ro)]
Jo=[Jo,Jo]
plot(ro,Jo,'ko')
xlabel('$r$')
axis tight
ylabel('$J(u^o(r),r;x_0)$')
leg={'$\alpha=0$','$\alpha=1$','$\alpha=10$','$\alpha=50$','$\alpha=100$','optimizer'}
legend(leg,'Location','North')
print('alpha','-depsc')
%% Effect of Cd
hCd=figure
load('10mode&Alpha10&Cd-Inf&mu-1iterInf.mat')
plot(r,J_fmin)
hold on
Jo=min(J_fmin)
ro=r(find(J_fmin==Jo))
ro=[ro,0.5+abs(0.5-ro)]
Jo=[Jo,Jo]
plot(ro,Jo,'ko','HandleVisibility','off')
load('10mode&Alpha10&Cd-5&mu-1iterInf.mat')
plot(r,J_fmin,'-.')
Jo=min(J_fmin)
ro=r(find(J_fmin==Jo))
ro=[ro,0.5+abs(0.5-ro)]
Jo=[Jo,Jo]
plot(ro,Jo,'ko','HandleVisibility','off')
load('10mode&Alpha10&Cd-4&mu-1iterInf.mat')
plot(r,J_fmin,'--')
Jo=min(J_fmin)
ro=r(find(J_fmin==Jo))
ro=[ro,0.5+abs(0.5-ro)]
Jo=[Jo,Jo]
plot(ro,Jo,'ko','HandleVisibility','off')
load('10mode&Alpha10&Cd-3&mu-1iterInf.mat')
plot(r,J_fmin,':')
Jo=min(J_fmin)
ro=r(find(J_fmin==Jo))
ro=[ro,0.5+abs(0.5-ro)]
Jo=[Jo,Jo]
plot(ro,Jo,'ko','HandleVisibility','off')
load('10mode&Alpha10&Cd-2&mu-1iterInf.mat')
plot(r,J_fmin,'.')
Jo=min(J_fmin)
ro=r(find(J_fmin==Jo))
ro=[ro,0.5+abs(0.5-ro)]
Jo=[Jo,Jo]
plot(ro,Jo,'ko')
xlabel('$r$')
axis tight
ylabel('$J(u^o(r),r;x_0)$')
leg={'$C_d=0$','$C_d=10^{-5}$','$C_d=10^{-4}$','$C_d=10^{-3}$','$C_d=10^{-2}$','optimizer'}
legend(leg,'Location','north')
print('Cd','-depsc')
%% Effect of mu
hmu=figure
load('10mode&Alpha10&Cd-4&mu-InfiterInf.mat')
plot(r,J_fmin)
hold on
Jo=min(J_fmin)
ro=r(find(J_fmin==Jo))
ro=[ro,0.5+abs(0.5-ro)]
Jo=[Jo,Jo]
plot(ro,Jo,'ko','HandleVisibility','off')
load('10mode&Alpha10&Cd-4&mu-2iterInf.mat')
plot(r,J_fmin,'-.')
Jo=min(J_fmin)
ro=r(find(J_fmin==Jo))
ro=[ro,0.5+abs(0.5-ro)]
Jo=[Jo,Jo]
plot(ro,Jo,'ko','HandleVisibility','off')
load('10mode&Alpha10&Cd-4&mu-1iterInf.mat')
plot(r,J_fmin,'--')
Jo=min(J_fmin)
ro=r(find(J_fmin==Jo))
ro=[ro,0.5+abs(0.5-ro)]
Jo=[Jo,Jo]
plot(ro,Jo,'ko','HandleVisibility','off')
load('10mode&Alpha10&Cd-4&mu0iterInf.mat')
plot(r,J_fmin,':')
Jo=min(J_fmin)
ro=r(find(J_fmin==Jo))
ro=[ro,0.5+abs(0.5-ro)]
Jo=[Jo,Jo]
plot(ro,Jo,'ko','HandleVisibility','off')
load('10mode&Alpha10&Cd-4&mu1iterInf.mat')
plot(r,J_fmin,'.')
Jo=min(J_fmin)
ro=r(find(J_fmin==Jo))
ro=[ro,0.5+abs(0.5-ro)]
Jo=[Jo,Jo]
plot(ro,Jo,'ko')
xlabel('$r$')
axis tight
ylabel('$J(u^o(r),r;x_0)$')
leg={'optimizer','$\mu=10$','$\mu=1$','$\mu=0.1$','$\mu=0.01$','$\mu=0$'}
legend(leg{6:-1:1},'Location','north')
print('mu','-depsc')
%% Parameters
fileID=fopen('parameters.txt','w');
p=['$EI=$%d, $\\rho a=$%d, $k=$%d, $l=$%d, $\\alpha=$%d, $\\delta=$%1.0g' ...
   ' $\\mu=$%1.0g, $C_d=$%1.0g, $c_w=$%d, $c_v=$%d, $c_u=$%d, $T=$%d'];
fprintf(fileID,p,[EI,rhoa,k,l,Alpha,delta,mu,Cd,cw,cv,cu,T]);
fclose(fileID);
