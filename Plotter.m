set(groot,'defaultLineLineWidth',1)
set(0,'defaultAxesFontSize',14)
set(groot,'defaulttextinterpreter','latex')
set(groot,'defaultAxesTickLabelInterpreter','latex')
set(groot,'defaultLegendInterpreter','latex');

%% Optimal Input vs Time
hu=figure;
%set(h,'LineColor','red')
plot(tspan,uo,'Color','black')
xlabel('$t$')
ylabel('$u^o(t)$')
title('Optimal input')
n=length(un)
for i=1:n
   hold on
   plot(tspan,un{i})
   drawnow
   pause(0.5)
end
plot(tspan,u_lqr,'--')
plot(tspan,u_fmin)
%% Optimal Cost
hc_comp=figure
plot(r,J_fmin,r,J_lqr)
legend('Nonlinear model','Linear model')
xlabel('Actuator location, $r$')
ylabel('Cost, $J(u^o,r;\textbf{z}_0)$')
%% Input Descent vs Time
hd=figure;
%set(h,'LineColor','red')
plot(tspan,uo,'Color','black')
xlabel('$t$')
ylabel('$D_uJ$')
title('Input Descent')
for i=1:n
   hold on
   plot(tspan,un{i})
   drawnow
   pause(0.5)
end
%% Cost vs Actuator Location (input is fixed)
hc=figure;
%set(h,'LineColor','red')
plot(r,J,'Color','black')
hold on
plot(r,J_fmin,'Color','red')
xlabel('Actuator location $r$')
ylabel('Cost $J$')
title('Cost')
%%

%% Mid-point deflection vs Time
hm=figure;
%set(h,'LineColor','red')
plot(tspan,wmo,'Color','black')
hold on
plot(tspan,wmo_lqr,'--')
plot(tspan,vmo,'.')
xlabel('$t$')
ylabel('$w(\frac{1}{2},t)$')
title('Mid-deflection')
