function [c,ce,grad_c,grad_ce]=Constraint_ux(ux,m,n)
x=ux(m+1:m+n);
c=x*x'-1;
grad_c=[zeros(1,m),2*x]';
ce=[];
grad_ce=[];
end