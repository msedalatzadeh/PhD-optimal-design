function r=proj(r0,rmin,rmax)
if r0<=rmin
   r=rmin;
elseif r0>=rmax
   r=rmax;
else
   r=r0;
end