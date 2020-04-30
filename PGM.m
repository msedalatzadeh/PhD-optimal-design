function [u,r,z,p,DuJ,DrJ]=PGM(u1,r1,par,linesearch,tolerance)
%par={[EI rhoa k l alpha delta mu Cd cw cv cu],tspan,z0}
%% Adjustments
%%% Initial step length
su0=5;
sr0=0;
min(par{1}(4)-r1,r1)/5;
%%
u{1}=u1;
r{1}=r1;
[z{1},p{1},DuJ,DrJ]=RTsolver(u{1},r{1},par);
hu{1}=-DuJ;
hr{1}=-DrJ;
J(1)=cost(u1,r1,par);
n=1;
tol=Inf;
while tol>tolerance
   %% Activate a line search method
   if strcmp(linesearch,'bisection')
      %%% Bisection
      [su{n},sr{n}]=bisection(u{n},r{n},hu{n},hr{n},par,su0,sr0);
      su0=su{n};
      sr0=sr{n};
   elseif strcmp(linesearch,'Wolfe')
      %%% Wolfe conditions
      [su{n},sr{n}]=Wolfe(u{n},r{n},DuJ,DrJ,hu{n},hr{n},par,su0,sr0);
      su0=su{n};
      sr0=sr{n};
   elseif strcmp(linesearch,'strongWolfe')
      %%% Strong Wolfe conditions
      [su{n},sr{n}]=strongWolfe(u{n},r{n},DuJ,DrJ,hu{n},hr{n},par,su0,sr0);
      su0=su{n};
      sr0=sr{n};
   elseif strcmp(linesearch,'secant')
      %%% Secant method
      [su{n},sr{n}]=secant(u{n},r{n},hu{n},hr{n},hu{n},hr{n},par,su0,sr0) ;
      su0=su{n};
      sr0=sr{n};
   end
   
   u{n+1}=u{n}+su{n}*hu{n};
   r{n+1}=r{n}+sr{n}*hr{n};
   [z{n+1},p{n+1},DuJ,DrJ]=RTsolver(u{n},r{n},par);
   hu{n+1}=-DuJ;
   hr{n+1}=-DrJ;
   J(n+1)=cost(u{n+1},r{n+1},par);
   disp([J(n+1) r{n+1}])
   tol=abs(J(n+1)-min(J(1:n)));
   n=n+1;
end