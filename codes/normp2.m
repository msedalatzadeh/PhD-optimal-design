function y=normp2(f)
syms EI Cd k l rhoa x;
y=int(EI*(diff(f(1),x,2))^2+k*f(1)^2+rhoa*f(2)^2,x,0,l);
end