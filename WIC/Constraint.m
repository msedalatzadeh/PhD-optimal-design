function [c,ce,grad_c,grad_ce]=Constraint(urx,m,n)
x=urx(m+2:m+1+n);
c=x*x'-1;
grad_c=[zeros(1,m+1),2*x]';
ce=[];
grad_ce=[];
end