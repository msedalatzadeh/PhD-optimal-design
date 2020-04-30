function [su,sr]=Wolfe(u,r,DuJ,DrJ,du,dr,par,su0,sr0)
J=cost(u,r,par);
su=su0;
sr=sr0;
c1=10^-4;
c2=0.1;
% % % c1=10^-2;
% % % c2=1;
n=1;
while n<5
   u1=u+su*du;
   r1=proj(r+sr*dr,0+par{1}(6),par{1}(4)-par{1}(6));
   J1=cost(u1,r1,par);
   [~,~,DuJ1,DrJ1]=RTsolver(u1,r1,par);
   SufficientDecrease=(J1<=J+c1*su*dot(DuJ,du)+c1*sr*dot(DrJ,dr));
   Curvature=(dot(DuJ1,du)+dot(DrJ1,dr)>=c2*dot(DrJ,dr)+c2*dot(DuJ,du));
   if SufficientDecrease&&Curvature
      break
   end
   if ~SufficientDecrease
      su=su/2;
      sr=sr/2;
   elseif ~Curvature
      su=2*su;
      sr=2*sr;
   end
   n=n+1;
end
end




% % % % % % % 
% % % % % % % 
% % % % % % % function [su,sr]=Wolfe(u,r,DuJ,DrJ,du,dr,par,su0,sr0)
% % % % % % % J=cost(u,r,par);
% % % % % % % su=su0;
% % % % % % % sr=sr0;
% % % % % % % c1=10^-4;
% % % % % % % c2=0.1;
% % % % % % % n=1;
% % % % % % % while n<4
% % % % % % %    u1=u+su*du;
% % % % % % %    r1=r+sr*dr;
% % % % % % %    J1=cost(u1,r1,par);
% % % % % % %    [~,~,DuJ1,DrJ1]=RTsolver(u1,r1,par);
% % % % % % %    SufficientDecrease=(J1<=J+c1*(su*dot(DuJ,du)+sr*dot(DrJ,dr)));
% % % % % % %    Curvature=(dot(DuJ1,du)>=c2*dot(DuJ,du))&(dot(DrJ1,dr)>=c2*dot(DrJ,dr));
% % % % % % %    if SufficientDecrease&&Curvature
% % % % % % %       break
% % % % % % %    end
% % % % % % %    if ~SufficientDecrease
% % % % % % %       su=su/2;
% % % % % % %       sr=sr/2;
% % % % % % %    elseif ~Curvature
% % % % % % %       su=2*su;
% % % % % % %       sr=2*sr;
% % % % % % %    end
% % % % % % %    n=n+1;
% % % % % % % end
% % % % % % % end