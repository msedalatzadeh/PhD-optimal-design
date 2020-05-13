function [su,sr]=secant(u,r,hu,hr,du,dr,par,su0,sr0)
[~,~,DuJ,~]=RTsolver(u+su0*du,r,par);
[~,~,~,DrJ]=RTsolver(u,r+sr0*dr,par);
denom_r=dot(DrJ+hr,dr);
denom_u=dot(DuJ+hu,du);
if denom_u==0 && denom_r~=0
   su=su0;
   sr=-dot(hr,dr)/denom_r;
elseif denom_r==0 && denom_u~=0
   su=-dot(hu,du)/denom_u;
   sr=sr0;
elseif denom_u==0 && denom_r==0
   su=su0;
   sr=sr0;
else
   su=-dot(hu,du)/denom_u;
   sr=-dot(hr,dr)/denom_r;
end