%% Generator
% This script generates the nonlinear functions appeared in the truncated
% nonlinear system for any order of approximation
tic
clc
clear all
N=20 %Specify the maximum order of approximation

syms alpha l x r delta EI rhoa k l par
assume(alpha>0 & l>0 & r>0 & EI>0 & rhoa>0 & k>0 & l>0 & delta>0)
assumeAlso(2*delta<l & delta<r & r+delta<l)
par=[EI,rhoa,k,l,alpha,delta];

b=piecewise(r-delta<x<r+delta,1/(2*delta),0);

cc=[];

for n=1:N/2
   cc=[cc,sym(sprintf('c%d',n))];
   for i=1:n
      B(2*i-1,1)=cc(i)*simplify(int(1/(2*delta)*sin(i*pi*x/l),x,r-delta,r+delta));
      B(2*i,1)=B(2*i-1,1);
   end
   
   B=subs(B,cc,c(1:n,par));
   matlabFunction(B,'File',sprintf('B%d',2*n),'Vars',{r,par});
   n
end
toc