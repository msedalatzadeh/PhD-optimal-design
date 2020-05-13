function y=An(N,par)
%par=[EI,rhoa,k,l,alpha,delta,mu,Cd,cw,cv,cu]
EI=par(1);
k=par(3);
l=par(4);
mu=par(7);
Cd=par(8);
y=[];
for n=1:(N/2)
   A=[-(mu*l^4 + Cd*pi^4*n^4)/(2*l^3),...
      (-mu*pi^2*l^4*n^2 + 2*k*l^4 - Cd*pi^6*n^6 + 2*EI*pi^4*n^4)/(2*l^3*n^2*pi^2);...
      -(mu*pi^2*l^4*n^2 + 2*k*l^4 + Cd*pi^6*n^6 + 2*EI*pi^4*n^4)/(2*l^3*n^2*pi^2), ...
      -(mu*l^4 + Cd*pi^4*n^4)/(2*l^3)];
   A=A*(c(n,par))^2;
   y=blkdiag(y,A);
end
end


