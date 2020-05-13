function [su,sr]=bisection(u,r,du,dr,par,su0,sr0)
J=cost(u,r,par);
su=su0;
sr=sr0;
while 1
   u1=u+su*du;
   r1=proj(r+sr*dr,0+par{1}(6),par{1}(4)-par{1}(6));
   J1=cost(u1,r1,par);
   
   if J1<=J
      break
   else
      su=su/2;
      sr=sr/2;
   end
end
end


% % % % % function [su,sr]=bisection(u,r,du,dr,par,su0,sr0)
% % % % % J=cost(u,r,par);
% % % % % su=su0;
% % % % % sr=sr0;
% % % % % while 1
% % % % %    u1=u+su*du;
% % % % %    r1=r+sr*dr;
% % % % %    J1=cost(u1,r1,par);
% % % % %    
% % % % %    if J1<=J
% % % % %       break
% % % % %    else
% % % % %       su=su/2;
% % % % %       sr=sr/2;
% % % % %    end
% % % % % end
% % % % % end