clc
clear all
tic
%% Specify the parameters of the railway track model
EI=1
rhoa=1
k=1
l=1
alpha=1
delta=0.001/2
mu=0.1;
Cd=0.0001;
cw=3
cv=1
cu=1
T=60
mesh=600
tspan=[0:0.03:15];
linspace(0,T,mesh);
u1=0.01*sin(tspan*pi/T);
r1=0.1*l;
hc=figure
hi=figure
%hc_lqr=figure
%hi_lqr=figure
parpool(8)
%% Initial conditions
z0=[]
for N=2
   z0=[z0;(-1)^(N/2)/(N/2)^2;1/(N/2)^2];
   par={[EI,rhoa,k,l,alpha,delta,mu,Cd,cw,cv,cu],tspan,z0}
   %%
   options=optimoptions('fmincon','Display','off','SpecifyObjectiveGradient'...
      ,true,'ScaleProblem',true,'MaxIterations',15,'UseParallel',true...
      ,'Algorithm','interior-point','SubproblemAlgorithm'...
      ,'cg')
   rmin=0+par{1}(6);
   rmax=par{1}(4)-par{1}(6);
   lb=[-10*ones(1,length(tspan)),rmin];
   ub=[+10*ones(1,length(tspan)),rmax];
   [ur,Cost]=fmincon(@(ur)RTcostgrad(ur,par),[u1,r1],[],[],[],[],lb,ub,[]...
      ,options);
   m=length(tspan);
   u_fmin=ur(1:m);
   disp(Cost)
   r_fmin=ur(m+1)
   
   
   J_fmin=zeros(1,21);
   parfor i=1:21
      r(i)=0+(i-1)/20*l
      %J_fmin(i)=cost(u_fmin,r(i),par);
      x=r(i);
      [~,J_fmin(i)]=fmincon(@(u)RTcostgrad_u(u,x,par),u_fmin,[],[],[],[]...
         ,lb(1:m),ub(1:m),[],options);
      %J_fmin(i)=y;
   end
   figure(hc)
   plot(r,J_fmin)
   xlabel('Actuator location, $r$')
   ylabel('Cost, $J(u^o,r;\textbf{z}_0)$')
   %title('Increasing Order of Approximation')
   Nth=num2ordinal(N)
   leg{N/2}=strcat(Nth,' order')
   legend(leg)
   hold on
   plot(r_fmin,Cost,'black*','HandleVisibility','off')
   grid on
   drawnow
   
   figure(hi)
   plot(tspan,u_fmin)
   xlabel('Time, $t$')
   ylabel('Optimal input, $u^o(t)$')
   %ylim([-0.5,0.5])
   legend(leg)
   %title('Input from Optimization')
   hold on
   drawnow
   
   
   %    A=An(N,par{1});
   %    Brfun=str2func(sprintf('B%d',N));
   %    Br=@(r)Brfun(r,par{1});
   %    Q=Qn(N,par{1});
   %    parfor i=1:101
   %       r(i)=0+(i-1)/100*l;
   %       [S,~,~]=care(A,Br(r(i)),Q,cu,[],[]);
   %       J_lqr(i)=z0'*S*z0;
   %    end
   %
   %
   %    J_diff=zeros(1,101);
   %    parfor i=1:101
   %       r(i)=0+(i-1)/100*l;
   %       B=Br(r(i));
   %       [~,~,K]=care(A,B,Q,cu,[],[]);
   %       u_lqr=zeros(1,m);
   %       for j=1:m
   %          u_lqr(j)=-K*expm((A-B*K)*tspan(j))*z0;
   %       end
   %       x=u_lqr(1:m);
   %       J_diff(i)=cost(x,r(i),par);
   %    end
   %
   %    figure(hc_lqr)
   %    plot(r,J_lqr)
   %    xlabel('$r$')
   %    title('Cost from ARE')
   %    axis auto
   %    hold on
   %    plot(r,J_diff,'-.')
   %    drawnow
   
   %    figure(hi_lqr)
   %    plot(tspan,u_lqr-u_fmin)
   %    xlabel('$t$')
   %    title('Input Difference ARE-Optimization')
   %    hold on
   %    drawnow
end

[zo,~,~,~]=RTsolver(u_fmin,r_fmin,par);
n=length(z0);
m=length(tspan);
wmo=zeros(1,m);
parfor i=1:m
   for j=1:(n/2)
      wmo(i)=wmo(i)+(zo(2*j-1,i)-zo(2*j,i))*c(j,par{1})*1/(j^2*pi^2)*sin(j*pi/2);
   end
end
[z,~,~,~]=RTsolver(rand(1,m),rand,par);
wm=zeros(1,m);
parfor i=1:m
   for j=1:(n/2)
      wm(i)=wm(i)+(z(2*j-1,i)-z(2*j,i))*c(j,par{1})*1/(j^2*pi^2)*sin(j*pi/2);
   end
end
hcomp=figure
plot(tspan,wmo,'green',tspan,wm,'red')
xlabel('Time, $t$')
%ylim([-0.06,0.06])
ylabel('Mid-deflection, $w(\frac{\ell}{2},t)$')
legend('Optimal','Random')
%title('Input from Optimization')
hold on
drawnow


save('workspace')
saveas(hi,'Input3','epsc')
saveas(hc,'Cost3','epsc')
saveas(hcomp,'Comparison3','epsc')