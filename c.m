function y=c(n,par)
EI=par(1);
rhoa=par(2);
k=par(3);
l=par(4);

y=sqrt((2*l^3*pi^4*n.^4)./(EI*pi^4*n.^4+k*l^4+rhoa*l^4*pi^4*n.^4));
end

