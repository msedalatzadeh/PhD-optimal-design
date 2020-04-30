function [Cost,Gradient]=RTcostgrad(ur,par)
%par={[EI rhoa k l alpha delta mu Cd cw cv cu],tspan,z0}
[~,~,DuJ,DrJ,Cost]=RTsolver(ur,par);
Gradient=[DuJ,DrJ];
end