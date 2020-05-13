clc
clear all
syms EI rhoa k Cd muu b
syms l n m lam x
assume(n,'integer')
assumeAlso(n>0)
assume(m,'integer')
assumeAlso(m>0)
assume(l>0)

%% Norm and Inner-Products
pi=sym(pi)
cn=sqrt((2*l^3*pi^4*n^4)/(EI*pi^4*n^4+k*l^4+rhoa*l^4*pi^4*n^4))
cm=sqrt((2*l^3*pi^4*m^4)/(EI*pi^4*m^4+k*l^4+rhoa*l^4*pi^4*m^4))

en=[1/(n^2*pi^2)*sin(n*pi*x/l),sin(n*pi*x/l)]
e_n=[-1/(n^2*pi^2)*sin(n*pi*x/l),sin(n*pi*x/l)]
em=cm*[1/(m^2*pi^2)*sin(m*pi*x/l),sin(m*pi*x/l)]
%Aem=cm*[sin(m*pi*x/l),-1/rhoa*(EI*diff(sin(m*pi*x/l)/(m^2*pi^2),x,4)+Cd*diff(sin(m*pi*x/l),x,4)+muu*sin(m*pi*x/l)+k*sin(m*pi*x/l)/(m^2*pi^2))]
Aem=stateop(em)
Aen=stateop(en)
Ae_n=stateop(e_n)

Qem=Qop(em)
Qen=Qop(en)
Qe_n=Qop(e_n)

normn=simplify(normp2(cn*e_n))

A=simplify([innerprod(Aen,en),innerprod(Ae_n,en);innerprod(Aen,e_n),innerprod(Ae_n,e_n)])
Q=simplify([innerprod(Qen,en),innerprod(Qe_n,en);innerprod(Qen,e_n),innerprod(Qe_n,e_n)])
%%
roots([rhoa,mu+(n*pi/l)^4*Cd,k+(n*pi/l)^4*EI])
%%
syms N
assume(N>0 & in(N,'integer'))
assumeAlso(n<N)
%%
syms m1 m2 m3
assume(m1>0 & in(m1,'integer'))
assume(m2>0 & in(m2,'integer'))
assume(m3>0 & in(m3,'integer'))

prod=int(sin(m1*pi*x/l)*sin(m2*pi*x/l)*sin(m3*pi*x/l)*sin(n*pi*x/l),x,0,l);
simplify(prod)

