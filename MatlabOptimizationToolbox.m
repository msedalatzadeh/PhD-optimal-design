clc
options=optimoptions('fmincon','Display','iter','SpecifyObjectiveGradient'...
   ,true,'ScaleProblem',true,'MaxIterations',10,'UseParallel',true,...
   'Algorithm','interior-point','SubproblemAlgorithm'...
   ,'factorization')
rmin=0+par{1}(6);
rmax=par{1}(4)-par{1}(6);
lb=[-10*ones(1,length(tspan)),rmin];
ub=[+10*ones(1,length(tspan)),rmax];
[ur,Cost]=fmincon(@(ur)RTcostgrad(ur,par),[u1,r1],[],[],[],[],lb,ub,[],options);
m=length(tspan);
u_fmin=ur(1:m);
r_fmin=ur(m+1)
J_fmin=Cost

parfor i=1:101
   r(i)=0+(i-1)/100*l;
   J_fmin(i)=cost(u_fmin,r(i),par);
end