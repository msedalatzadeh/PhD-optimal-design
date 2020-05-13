function y=innerprod(f1,f2)
syms EI Cd k l rhoa x;
y=int(EI*diff(f1(1),x,2)*diff(f2(1),x,2)+k*f1(1)*f2(1)+rhoa*f1(2)*f2(2),x,0,l);
end