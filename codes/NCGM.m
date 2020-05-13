function [u,r,z,p,DuJ,DrJ]=NCGM(u1,r1,par,linesearch,steplength,tolerance)
%par={[EI rhoa k l alpha delta mu Cd cw cv cu],tspan,z0}
%% Adjustments
%%% Initial step length
su0=0.1;
sr0=0.1;
min(par{1}(4)-r1,r1)/5;
%%
u{1}=u1;
r{1}=r1;
[z{1},p{1},DuJ{1},DrJ{1}]=RTsolver(u1,r1,par);
hu{1}=-DuJ{1};
hr{1}=-DrJ{1};
du{1}=hu{1};
dr{1}=hr{1};
J(1)=cost(u1,r1,par);
n=1;
tol=Inf;
while n<20
   %% Activate a line search method
   if strcmp(linesearch,'bisection')
      %%% Bisection
      [su{n},sr{n}]=bisection(u{n},r{n},du{n},dr{n},par,su0,sr0);
      su0=su{n};
      sr0=sr{n};
   elseif strcmp(linesearch,'Wolfe')
      %%% Wolfe conditions
      [su{n},sr{n}]=Wolfe(u{n},r{n},DuJ{n},DrJ{n},du{n},dr{n},par,su0,sr0);
      su0=su{n};
      sr0=sr{n};
   elseif strcmp(linesearch,'strongWolfe')
      %%% Strong Wolfe conditions
      [su{n},sr{n}]=strongWolfe(u{n},r{n},DuJ,DrJ,du{n},dr{n},par,su0,sr0);
      su0=su{n};
      sr0=sr{n};
   elseif strcmp(linesearch,'secant')
      %%% Secant method
      [su{n},sr{n}]=secant(u{n},r{n},hu{n},hr{n},du{n},dr{n},par,su0,sr0) ;
      su0=su{n};
      sr0=sr{n};
   end
   %%
   
   u{n+1}=u{n}+su{n}*du{n};
   r{n+1}=proj(r{n}+sr{n}*dr{n},0+par{1}(6),par{1}(4)-par{1}(6));
   [z{n+1},p{n+1},DuJ{n+1},DrJ{n+1}]=RTsolver(u{n+1},r{n+1},par);
   hu{n+1}=-DuJ{n+1};
   hr{n+1}=-DrJ{n+1};
   
   %% Activate a step length formula
   if strcmp(steplength,'FR')
      beta{n+1}=(norm(hu{n+1},2)+norm(hr{n+1},2))/...
         (norm(hu{n},2)+norm(hr{n},2));
   elseif strcmp(steplength,'PR')
      betau{n+1}=dot(hu{n+1},hu{n+1}-hu{n})/norm(hu{n},2);
      betar{n+1}=dot(hr{n+1},hr{n+1}-hr{n})/norm(hr{n},2);
   elseif strcmp(steplength,'HS')
      betau{n+1}=dot(hu{n+1},hu{n+1}-hu{n})/dot(du{n},hu{n+1}-hu{n});
      betar{n+1}=dot(hr{n+1},hr{n+1}-hr{n})/dot(dr{n},hr{n+1}-hr{n});
   elseif strcmp(steplength,'HZ')
      gammau{n+1}=hu{n+1}-hu{n};
      betabaru{n+1}=dot(gammau{n+1}-2*norm(gammau{n+1},2)^2/dot(du{n},gammau{n+1})*du{n},hu{n+1})/dot(du{n+1},gammau{n+1});
      etau{n+1}=-1/norm(du{n},2)*min(0.01,norm(hu{n},1));
      betau{n+1}=max(betabaru{n+1},etau{n+1});
   end
   
   du{n+1}=hu{n+1}+beta{n+1}*du{n};
   dr{n+1}=hr{n+1}+beta{n+1}*dr{n};
   
   J(n+1)=cost(u{n+1},r{n+1},par);
   disp([J(n+1) r{n+1}])
   tol=abs(J(n+1)-min(J(1:n)));
   n=n+1;
end



% % % % % % function [u,r,z,p,DuJ,DrJ]=NCGM(u1,r1,par,linesearch,steplength,tolerance)
% % % % % % par={[EI rhoa k l alpha delta mu Cd cw cv cu],tspan,z0}
% % % % % % Adjustments
% % % % % % Initial step length
% % % % % % su0=5;
% % % % % % sr0=0;
% % % % % % min(par{1}(4)-r1,r1)/5;
% % % % % % 
% % % % % % u{1}=u1;
% % % % % % r{1}=r1;
% % % % % % [z{1},p{1},DuJ,DrJ]=RTsolver(u1,r1,par);
% % % % % % hu{1}=-DuJ;
% % % % % % hr{1}=-DrJ
% % % % % % du{1}=hu{1};
% % % % % % dr{1}=hr{1};
% % % % % % J(1)=cost(u1,r1,par);
% % % % % % n=1;
% % % % % % tol=Inf;
% % % % % % while tol>tolerance
% % % % % %    Activate a line search method
% % % % % %    if strcmp(linesearch,'bisection')
% % % % % %       Bisection
% % % % % %       [su{n},sr{n}]=bisection(u{n},r{n},du{n},dr{n},par,su0,sr0);
% % % % % %       su0=su{n};
% % % % % %       sr0=sr{n};
% % % % % %    elseif strcmp(linesearch,'Wolfe')
% % % % % %       Wolfe conditions
% % % % % %       [su{n},sr{n}]=Wolfe(u{n},r{n},DuJ,DrJ,du{n},dr{n},par,su0,sr0);
% % % % % %       su0=su{n};
% % % % % %       sr0=sr{n};
% % % % % %    elseif strcmp(linesearch,'strongWolfe')
% % % % % %       Strong Wolfe conditions
% % % % % %       [su{n},sr{n}]=strongWolfe(u{n},r{n},DuJ,DrJ,du{n},dr{n},par,su0,sr0);
% % % % % %       su0=su{n};
% % % % % %       sr0=sr{n};
% % % % % %    elseif strcmp(linesearch,'secant')
% % % % % %       Secant method
% % % % % %       [su{n},sr{n}]=secant(u{n},r{n},hu{n},hr{n},du{n},dr{n},par,su0,sr0) ;
% % % % % %       su0=su{n};
% % % % % %       sr0=sr{n};
% % % % % %    end
% % % % % %    
% % % % % %    
% % % % % %    u{n+1}=u{n}+su{n}*du{n};
% % % % % %    r{n+1}=r{n}+sr{n}*dr{n};
% % % % % %    [z{n+1},p{n+1},DuJ,DrJ]=RTsolver(u{n+1},r{n+1},par);
% % % % % %    hu{n+1}=-DuJ;
% % % % % %    hr{n+1}=-DrJ;
% % % % % %    
% % % % % %    Activate a step length formula
% % % % % %    if strcmp(steplength,'FR')
% % % % % %       betau{n+1}=norm(hu{n+1},2)/norm(hu{n},2);
% % % % % %       betar{n+1}=norm(hr{n+1},2)/norm(hr{n},2);
% % % % % %    elseif strcmp(steplength,'PR')
% % % % % %       betau{n+1}=dot(hu{n+1},hu{n+1}-hu{n})/norm(hu{n},2);
% % % % % %       betar{n+1}=dot(hr{n+1},hr{n+1}-hr{n})/norm(hr{n},2);
% % % % % %    elseif strcmp(steplength,'HS')
% % % % % %       betau{n+1}=dot(hu{n+1},hu{n+1}-hu{n})/dot(du{n},hu{n+1}-hu{n});
% % % % % %       betar{n+1}=dot(hr{n+1},hr{n+1}-hr{n})/dot(dr{n},hr{n+1}-hr{n});
% % % % % %    elseif strcmp(steplength,'HZ')
% % % % % %       gammau{n+1}=hu{n+1}-hu{n};
% % % % % %       betabaru{n+1}=dot(gammau{n+1}-2*norm(gammau{n+1},2)^2/dot(du{n},gammau{n+1})*du{n},hu{n+1})/dot(du{n+1},gammau{n+1});
% % % % % %       etau{n+1}=-1/norm(du{n},2)*min(0.01,norm(hu{n},1));
% % % % % %       betau{n+1}=max(betabaru{n+1},etau{n+1});
% % % % % %    end
% % % % % %    
% % % % % %    du{n+1}=hu{n+1}+betau{n+1}*du{n};
% % % % % %    dr{n+1}=hr{n+1}+betar{n+1}*dr{n};
% % % % % %    
% % % % % %    J(n+1)=cost(u{n+1},r{n+1},par);
% % % % % %    disp([J(n+1) r{n+1}])
% % % % % %    tol=abs(J(n+1)-min(J(1:n)));
% % % % % %    n=n+1;
% % % % % % end