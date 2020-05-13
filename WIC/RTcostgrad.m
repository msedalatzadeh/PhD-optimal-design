function [Cost,Gradient]=RTcostgrad(urx,par)
%par={[EI rhoa k l alpha delta mu Cd cw cv cu],tspan,z0}
[~,~,DuJ,DrJ,DxJ,Cost]=RTsolver(urx,par);
Gradient=[DuJ,DrJ,DxJ];
end