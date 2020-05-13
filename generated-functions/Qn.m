function y=Qn(N,par)
%par=[EI,rhoa,k,l,alpha,delta,mu,Cd,cw,cv,cu]
EI=par(1);
k=par(3);
rhoa=par(2);
l=par(4);
cw=par(9);
cv=par(10);
y=[];
for n=1:(N/2)
 Q=[(cv*rhoa*pi^4*l^4*n^4 + cw*k*l^4 + EI*cw*pi^4*n^4)/(2*l^3*n^4*pi^4),...
    -(-cv*rhoa*pi^4*l^4*n^4 + cw*k*l^4 + EI*cw*pi^4*n^4)/(2*l^3*n^4*pi^4);...
-(-cv*rhoa*pi^4*l^4*n^4 + cw*k*l^4 + EI*cw*pi^4*n^4)/(2*l^3*n^4*pi^4),...
   (cv*rhoa*pi^4*l^4*n^4 + cw*k*l^4 + EI*cw*pi^4*n^4)/(2*l^3*n^4*pi^4)];
 
   Q=Q*(c(n,par))^2;
   % %    q1=c(i,par)*(cv-cw)*rhoa*n^4*l^4*pi^4+cw/(c(i,par))^2;
   % %    q2=c(i,par)*(cv+cw)*rhoa*n^4*l^4*pi^4-cw/(c(i,par))^2;
   % %    Q=[q1,q2;q2,q1];
   y=blkdiag(y,Q);
end
end