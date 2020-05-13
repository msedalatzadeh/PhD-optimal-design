function y=stateop(f)
syms EI Cd k rhoa x muu;
y(1,1)=f(2);
y(1,2)=-1/rhoa*(EI*diff(f(1),x,4)+Cd*diff(f(2),x,4)+k*f(1)+muu*f(2));
end