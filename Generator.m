%% Generator
% This script generates the nonlinear functions appeared in the truncated
% nonlinear system for any order of approximation
tic
clc
clear all
N=20  %Specify the maximum order of approximation

syms alpha l x r delta EI rhoa k l par
assume(alpha>0 & l>0 & r>0 & EI>0 & rhoa>0 & k>0 & l>0 & delta>0)
assumeAlso(2*delta<l & delta<r & r+delta<l)
par=[EI,rhoa,k,l,alpha,delta];

b=piecewise(r-delta<x<r+delta,(x-r+delta)^2*(x-r-delta)^2/(16*delta^5)*15,0);
br=diff(b,r);
%%
z=[];
cc=[];
W=0;

for n=1:N
   z=[z;sym(sprintf('z%d',n))];
end
for n=1:N/2
   cc=[cc,sym(sprintf('c%d',n))];
   W=W+z(2*n-1)*cc(n)*sin(n*pi*x/l)/(n^2*pi^2)...
      -z(2*n)*cc(n)*sin(n*pi*x/l)/(n^2*pi^2);
   for i=1:n
      F(2*i-1,1)=-alpha*cc(i)*simplify(int(W^3*sin(i*pi*x/l),x,0,l));
      F(2*i,1)=F(2*i-1,1);
      
      B(2*i-1,1)=cc(i)*simplify(int(b*sin(i*pi*x/l),x,0,l));
      B(2*i,1)=B(2*i-1,1);
      
      dB(1,2*i-1)=cc(i)*simplify(int(br*sin(i*pi*x/l),x,0,l));
      dB(1,2*i)=dB(1,2*i-1);
   end
   
   for i=1:n
      for j=1:n
         dF(2*i-1,2*j-1)=-3*alpha*cc(i)*cc(j)*simplify(int(W^2*sin(i*pi*x/l)*sin(j*pi*x/l),x,0,l));
         dF(2*i,2*j-1)=dF(2*i-1,2*j-1);
         dF(2*i-1,2*j)=dF(2*i-1,2*j-1);
         dF(2*i,2*j)=dF(2*i-1,2*j-1);
      end
   end
   F=subs(F,cc,c(1:n,par));
   dF=subs(dF,cc,c(1:n,par));
   B=subs(B,cc,c(1:n,par));
   dB=subs(dB,cc,c(1:n,par));
   
   matlabFunction(F,'File',sprintf('F%d',2*n),'Vars',{z,par});
   matlabFunction(dF,'File',sprintf('dF%d',2*n),'Sparse',true,'Vars',{z,par});
   matlabFunction(B,'File',sprintf('B%d',2*n),'Vars',{r,par});
   matlabFunction(dB,'File',sprintf('dB%d',2*n),'Vars',{r,par});
   n
end
toc