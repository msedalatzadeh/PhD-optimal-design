function [su,sr]=strongWolfe(u,r,DuJ,DrJ,du,dr,par,su0,sr0)
J=cost(u,r,par);
su=su0;
sr=sr0;
c1=10^-4;
c2=0.1;
n=1;
while n<4
   u1=u+su*du;
   r1=r+sr*dr;
   J1=cost(u1,r1,par);
   [~,~,DuJ1,DrJ1]=RTsolver(u1,r1,par);
   SufficientDecrease=(J1<=J+c1*(su*dot(DuJ,du)+sr*dot(DrJ,dr)));
   Curvature=(abs(dot(DuJ1,du))<=c2*abs(dot(DuJ,du)))&...
              abs(dot(DrJ1,dr))<=c2*abs(dot(DrJ,dr));
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